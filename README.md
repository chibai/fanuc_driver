# fanuc_driver

C++23 机械臂驱动库，提供品牌无关的抽象基类 `Robot_driver`，
目前实现的品牌驱动为 FANUC Remote Motion Interface（RMI）的 `Fanuc_driver`。
`Fanuc_driver` 公开继承 `Robot_driver`，只提供协议无关接口；
RMI 数据类型和通信实现通过 PImpl 隐藏，不随安装包发布。

本次为不兼容重构：移除 Stream Motion、GPIO 缓冲和 YAML 配置、Eigen、
`fanuc_client`、单例以及旧 `fanuc_driver` / `RMIConnection` 接口，不保留兼容层。
连接使用标准 RMI `FRC_Connect`，不再使用 `FRC_Connect_STMO`。

## 目录

```text
include/robot_driver/
├── robot_driver.hpp             顶层聚合入口
├── core/
│   ├── robot.hpp                Robot_driver 品牌无关契约
│   ├── types.hpp                RobotStatus
│   └── error.hpp                ResponseCode、ResponseStatus、Response<T>
└── drivers/fanuc/
    └── robot.hpp                Fanuc_driver 公开接口，仅依赖 core
src/
├── core/error.cpp               通用状态消息
└── drivers/fanuc/
    ├── robot.cpp                PImpl 和通用接口适配
    ├── rmi_driver.cpp           内部 TCP 通信和 RMI 实现
    ├── serialization.cpp
    ├── deserialization.cpp
    └── detail/                  packets.hpp、types.hpp、内部驱动及协议辅助头文件
tests/
├── CMakeLists.txt
├── core/                        契约和状态测试
├── drivers/fanuc/               模拟控制器和协议测试
└── consumer/                    安装包独立消费者
examples/
├── CMakeLists.txt
└── connect_fanuc.cpp             仅连接和查询，不发送运动或 I/O 写命令
cmake/                           安装包配置
docs/                            API 和分层说明
LICENSES/                        保留原有许可证文本
```

完整的公开接口、参数和数据类型说明见 [接口文档](docs/API.md)。
目录依赖边界见 [架构说明](docs/architecture.md)。只创建已有实现对应的目录，
暂不提供其他品牌、型号、运动学或规划模块。FANUC 仍使用 RMI，不是 FOCAS。
保留 `Robot_driver`、`Fanuc_driver` 的名称及
`find_package(fanuc_driver 3)` / `fanuc_driver::fanuc_driver` 构建契约。
RMI 协议包、运动和扩展接口现仅供内部实现使用，不再是公开 API；
品牌无关代码应包含 `<robot_driver/core/robot.hpp>`，聚合入口为 `<robot_driver/robot_driver.hpp>`。

## 构建

需要支持 C++23（含 `std::expected`）的编译器与标准库，例如 GCC 13+、
CMake 3.22+、Boost 1.83+ 开发头文件及 nlohmann/json 3.11+。运行环境需有 RMI 功能的控制器。
网络使用 Boost.Asio 和 nlohmann/json；不再依赖 reflect-cpp、sockpp、Eigen 或 YAML。
C++23 不包含 `std::asio`；本库采用 `boost::asio`，且不在公开头文件中暴露 Boost 类型。
内部通信使用 `co_spawn` / `awaitable` 协程和 `as_tuple(use_awaitable)` 显式处理网络错误；
设计依据及 RMI 超时、窗口和生命周期约束见[架构说明](docs/architecture.md#asio-协程设计)。

```bash
# Ubuntu / Debian
sudo apt-get install cmake g++ libboost-dev nlohmann-json3-dev
cmake -S . -B build -DBUILD_TESTING=ON -DBUILD_EXAMPLES=ON
cmake --build build -j2
ctest --test-dir build --output-on-failure
cmake --install build --prefix "$PWD/install"
```

### VS Code（Ubuntu 26.04 / G++ 15.2 / C++23）

仓库提供完整的 `.vscode/` 配置，使用系统的 `/usr/bin/g++-15`、Ninja 和 GDB。
先安装依赖，并确认编译器版本为预期的 15.2（配置固定 GCC 主版本，不锁定补丁版本）：

```bash
sudo apt-get update
sudo apt-get install g++-15 cmake ninja-build gdb clang-format libboost-dev nlohmann-json3-dev
/usr/bin/g++-15 --version
code .
```

首次打开时安装工作区推荐扩展 **C/C++** 和 **CMake Tools**；
通过 `CMake: Select a Kit` 选择 **GCC 15**，通过 `CMake: Select Variant` 选择 **Debug**。
自动配置会启用测试和示例；首次配置测试需要联网下载 GoogleTest。

| 操作 | 入口 |
|---|---|
| Debug 构建（自动先配置） | `Ctrl+Shift+B` |
| 构建并运行本地测试 | `Tasks: Run Task` → `CMake: test Debug` |
| Release 构建（自动先配置） | `Tasks: Run Task` → `CMake: build Release` |
| 断点调试本地测试 | 运行和调试中选择 `Debug local tests (GDB)`，按 `F5` |
| 调试状态查询示例 | 选择 `Debug status-only example (controller connection)`，输入控制器 IP |
| 单个测试发现与运行 | CMake 配置和构建完成后，在测试资源管理器中运行 |

Debug / Release 产物分别位于 `build/vscode/Debug` 和 `build/vscode/Release`。
IntelliSense 由 CMake Tools 提供真实编译参数，无需手工填写系统或 Boost 头文件路径；
保存 C/C++ 文件时使用系统 `clang-format` 和仓库的 `.clang-format`。
不要同时启用 clangd 的语言服务，以免与 C/C++ 扩展重复诊断。

默认 F5 调试仅使用本地模拟控制器的测试，不连接真实机器人。
状态查询示例会实际连接输入的控制器，但不发送运动或 I/O 写命令；启动后先停在入口。
不要将调试器暂停或退出视为机器人安全停止措施。

## 使用

```cmake
find_package(fanuc_driver 3 REQUIRED)
target_link_libraries(my_app PRIVATE fanuc_driver::fanuc_driver)
```

### 多品牌统一接口

业务代码依赖 `Robot_driver&` 或 `std::unique_ptr<Robot_driver>`，无需依赖 RMI 数据类型。
控制方法返回 `ResponseStatus`，状态查询返回 `Response<RobotStatus>`；
数字输入返回 `std::expected<bool, ResponseStatus>`。读取失败通过 `error()` 获取通用错误，
读取成功通过 `value()` 或解引用获取数据（`false` 也是成功结果）。

```cpp
#include <robot_driver/drivers/fanuc/robot.hpp>

ResponseStatus queryRobot(Robot_driver& robot)
{
  const auto connected = robot.connect();
  if (!connected.ok())
    return connected;
  const auto status = robot.getStatus();
  if (!status.ok())
    return status.status;
  // status.data->servo_ready / status.data->speed_override
  return robot.disconnect();
}

int main()
{
  Fanuc_driver fanuc("192.168.1.100");
  const auto result = queryRobot(fanuc);
  return result.ok() ? 0 : result.code();
}
```

基类统一了连接、断开、运动初始化、暂停、继续、中止、复位、速度倍率、状态、
数字输入读取和数字输出写入。新增品牌时继承 `Robot_driver` 并实现全部 `do*` 纯虚方法，
在品牌驱动内负责协议转换和错误检查，业务代码无需修改。
统一接口的超时以秒计，默认 1 秒，负值无限等待；速度倍率以百分比计。
状态的 `servo_ready` 表示伺服就绪，`speed_override` 表示速度倍率百分比。
品牌驱动不支持某操作时应返回 `INTERFACE_NOTIMPLEMENTED`，不得静默忽略或伪造成功。

基类采用非虚公开入口调用纯虚 `do*` 方法（NVI）。
直接通过 `Fanuc_driver` 或通过基类调用，均获得相同的通用状态和数据。
公开控制方法返回 `ResponseStatus`，状态查询返回 `Response<RobotStatus>`，
数字输入读取返回 `std::expected<bool, ResponseStatus>`，不包含 RMI 数据或错误码。
FANUC 专用的运动报文、构型、坐标系、寄存器、系统变量等实现保留在内部驱动中，
不再通过公开类暴露；本次不新增这些功能的品牌无关数据模型。
基类具有虚析构函数，禁止复制和移动；析构不发送停止或断开命令。
本次返回类型和错误处理方式不兼容旧接口，升级时需修改调用代码并重新编译下游程序。

### 直接使用 FANUC 驱动

```cpp
#include <robot_driver/drivers/fanuc/robot.hpp>

int main()
{
  Fanuc_driver driver("192.168.1.100");  // 默认 RMI 端口 16001
  const auto connection = driver.connect();
  if (!connection.ok())
    return connection.code();
  const auto status = driver.getStatus();
  if (!status.ok())
    return status.status.code();
  // status.data->servo_ready / status.data->speed_override
  return driver.disconnect().code();
}
```

构造函数打开初始 TCP 连接；`connect()` 执行 RMI 握手，并连接控制器返回的会话端口。
状态查询不自动初始化运动，也不会读取 Stream Motion 变量或运动限制。
公开接口不提供运动报文发送；运动初始化使用 `initializeRemoteMotion()`，中止使用 `abort()`，
结束会话使用 `disconnect()`。析构仅释放 TCP 资源，不隐式发送停止或复位命令，
不能作为机器人安全停止机制。

每个实例独立持有连接和序列号，不可复制或移动。请在单个应用线程中调用同一实例，
库不提供后台轮询线程或实时流控制。响应值均按值返回，不依赖内部缓存视图。
内部使用 Asio 异步读写和 `io_context`，在调用线程中驱动完成：
同步响应等待使用 `steady_clock` 截止时间，不进行忙轮询；`check*` 仅处理当前可用数据。
组合写操作发送完整 CRLF 帧，接收保留半包并拆分粘包。响应超时不会取消待接收数据，
后续调用仍可继续收取响应。连接切换会取消旧连接的读取并清除其半包。
实现使用 RAII、`std::expected` 表达 I/O 完成结果、`std::span` 参数视图、
C++23 `std::views::zip` 映射程序参数，以及 ranges 投影查找和移动传递响应。

超时参数以秒为单位，省略或 `std::nullopt` 为 1 秒，负数表示无限等待。
超时必须为有限值；NaN 和无穷大返回 `INVALID_PARAMETER`。
同步调用的网络断开、响应超时和控制器非零 `ErrorID` 分别返回 `CONNECTION_ERROR`、
`WAITING_TIMEOUT`、`CUSTOM_ERROR`（9999）。`Response<T>` 失败时 `data` 为空；
数字输入读取的 `std::expected` 失败时没有值，通过 `error()` 获取 `ResponseStatus`。
`response.status.code()` 提供统一数值码，`message()` 提供用户消息，`detail` 保留诊断详情，
RMI 响应中的非零 FANUC `ErrorID` 统一映射为 `CUSTOM_ERROR`（9999），
错误说明保存在 `detail`，不再单独返回原始 RMI 错误码。
消息读取 `ROBOTDRIVERCORE_LANGUAGE=zh/en/de`，默认或未知语言回退中文，
也可显式调用 `message("en")`；翻译随库编译，无需部署外部 CSV。
构造函数仍立即建连并可能抛异常。
资源分配失败和程序逻辑错误不会被包装为普通操作状态。

```cpp
const auto input = driver.readDigitalInputPort(1);
if (input)
  std::cout << *input << '\n';
else
  std::cerr << input.error().code() << ": " << input.error().message() << '\n';
```

`std::expected` 的布尔转换表示“是否读取成功”，不代表读取到的电平或数值。
仅在失败时调用 `error()`，仅在成功时解引用。
旧协议返回值需迁移为上述通用返回值；原有协议专用扩展不属于公开接口。

示例不发送运动或 I/O 写命令：

```bash
./build/fanuc_driver_example 192.168.1.100
```

## 公开功能

| 功能 | `Fanuc_driver` 方法 |
|---|---|
| 连接、端口协商、断开 | `connect`、`disconnect` |
| 运动初始化 | `initializeRemoteMotion` |
| 暂停、继续、中止、复位、速度倍率 | `pause`、`resume`、`abort`、`reset`、`setSpeedOverride` |
| 伺服就绪、速度倍率查询 | `getStatus` |
| 数字输入读取、数字输出写入 | `readDigitalInputPort`、`writeDigitalOutputPort` |

## 内部协议实现

以下功能保留在 `src/drivers/fanuc/detail/rmi_driver.hpp` 的 `FanucRmiDriver` 中，
用于协议实现和内部测试，**不是 `Fanuc_driver` 的公开成员**，安装包不能调用。

| 功能 | `FanucRmiDriver` 内部方法 |
|---|---|
| RMI 连接、端口协商、断开 | `connect`、`disconnect` |
| 运动初始化（含 GroupMask / RTSA / PLTZMODE） | `initializeRemoteMotion` |
| 暂停、继续、中止、复位、速度倍率 | `pause`、`resume`、`abort`、`reset`、`setSpeedOverride` |
| 控制器状态、扩展状态、报警文本 | `getStatus`、`getExtendedStatus`、`readError` |
| 关节角、笛卡尔位置、TCP 速度 | `readJointAngles`、`getCartesianPosition`、`getTCPSpeed` |
| 数值寄存器、位置寄存器读写 | `readNumericRegister`、`writeNumericRegister`、`readPositionRegister`、`writePositionRegister` |
| 数字输入读取、数字输出写入 | `readDigitalInputPort`、`writeDigitalOutputPort` |
| 通用 I/O 读写（类型由控制器支持） | `readIOPort`、`writeIOPort` |
| 系统变量读写 | `readVariablePacket`、`writeVariablePacket` |
| 当前用户/工具坐标系选择及数据读写 | `getUFrameUTool`、`setUFrameUTool`、`readUFrameData`、`writeUFrameData`、`readUToolData`、`writeUToolData` |
| 负载表选择、负载参数和补偿参数读写 | `getPayloadSchedule`、`setPayloadSchedule`、`getPayloadValue`、`setPayloadValue`、`getPayloadComp`、`setPayloadComp` |
| TP 程序调用（最多 10 个参数） | `programCall`、`programCallNonBlocking` |
| 同步关节运动（关节表示） | `sendJointMotion` |
| 同步/异步运动与队列指令 | `sendRMIPacket<Packet>`、`sendRMIPacketNonBlocking` |
| 异步指令响应、剩余指令窗口 | `getLastInstructionResponse`、`getRemainingBufferSize` |
| 系统故障、超时终止、通信和未知包查询 | `checkSystemFault`、`checkTimeoutTerminate`、`checkCommunicationPacket`、`checkUnknownPacket` |
| 错误码说明、文本编码设置 | `getErrorMessageString`、`setEncoding` |

运动与队列指令支持以下 `rmi::*Packet`，可使用同步模板或异步重载：

| 指令 | Packet 类型 |
|---|---|
| 关节绝对/相对运动 | `JointMotionPacket`、`JointRelativePacket`、`JointMotionJRepPacket`、`JointRelativeJRepPacket` |
| 直线绝对/相对运动 | `LinearMotionPacket`、`LinearRelativePacket`、`LinearMotionJRepPacket`、`LinearRelativeJRepPacket` |
| 圆弧绝对/相对运动 | `CircularMotionPacket`、`CircularRelativePacket` |
| 样条运动 | `SplineMotionPacket`、`SplineMotionJRepPacket` |
| 等待数字输入、等待时间 | `WaitForDINPacket`、`WaitForTimePacket` |
| 队列内切换用户坐标系、工具坐标系、负载表 | `SetUFramePacket`、`SetToolFramePacket`、`SetPayloadInstructionPacket` |

同步调用等待对应序列号的响应；异步调用只发送并分配序列号，不等待执行完成。
程序调用超过 10 个参数时，同步接口返回 `INVALID_PARAMETER`，异步接口在发送前抛出
`std::invalid_argument`，不再静默丢弃多余参数。
通过 `getLastInstructionResponse()` 主动接收并检查最新响应的 `SequenceID` 与 `ErrorID`。
`getRemainingBufferSize()` 按 8 条指令窗口计算，不会自动限流或阻止发送；
初始化会重置序列号。关节角采用控制器原生的度，位置采用毫米，
速度单位由请求的 `SpeedType` 决定，不做弧度或耦合关节转换。

**运动和写入接口会改变真实控制器状态。请确认安全条件后使用，不要将通信 API 当作安全功能。**
