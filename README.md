# fanuc_driver

仅包含 FANUC Remote Motion Interface（RMI）的 C++23 驱动库。
唯一对外功能入口为 `Fanuc_driver` 类；协议数据类型保留在 `rmi` 命名空间，
不需要创建其他连接对象。

本次为不兼容重构：移除 Stream Motion、GPIO 缓冲和 YAML 配置、Eigen、
`fanuc_client`、单例以及旧 `fanuc_driver` / `RMIConnection` 接口，不保留兼容层。
连接使用标准 RMI `FRC_Connect`，不再使用 `FRC_Connect_STMO`。

## 目录

```text
include/fanuc_driver/
  fanuc_driver.hpp      Fanuc_driver 公开接口
  packets.hpp           RMI 请求、响应和运动数据
  types.hpp             指令响应和程序调用参数
src/
  fanuc_driver.cpp      TCP 通信、RMI 会话和功能实现
  serialization.cpp    请求序列化
  deserialization.cpp  响应解析
  detail/               内部序列化声明和 TCP 分帧
tests/                  协议单元测试和本地模拟控制器测试
examples/               仅查询状态的安全示例
cmake/                  安装包配置
```

完整的公开接口、参数和数据类型说明见 [接口文档](docs/API.md)。

## 构建

需要支持 C++23（含 `std::expected`）的编译器与标准库，例如 GCC 13+、
CMake 3.22+、Boost 1.74+ 开发头文件及 nlohmann/json 3.11+。运行环境需有 RMI 功能的控制器。
网络使用 Boost.Asio 和 nlohmann/json；不再依赖 reflect-cpp、sockpp、Eigen 或 YAML。
C++23 不包含 `std::asio`；本库采用 `boost::asio`，且不在公开头文件中暴露 Boost 类型。

```bash
# Ubuntu / Debian
sudo apt-get install cmake g++ libboost-dev nlohmann-json3-dev
cmake -S . -B build -DBUILD_TESTING=ON -DBUILD_EXAMPLES=ON
cmake --build build -j2
ctest --test-dir build --output-on-failure
cmake --install build --prefix "$PWD/install"
```

## 使用

```cmake
find_package(fanuc_driver 3 REQUIRED)
target_link_libraries(my_app PRIVATE fanuc_driver::fanuc_driver)
```

```cpp
#include <fanuc_driver/fanuc_driver.hpp>

int main()
{
  Fanuc_driver driver("192.168.1.100");  // 默认 RMI 端口 16001
  const auto connection = driver.connect();
  const auto status = driver.getStatus();
  const auto joints = driver.readJointAngles();
  driver.disconnect();
}
```

构造函数打开初始 TCP 连接；`connect()` 执行 RMI 握手，并连接控制器返回的会话端口。
状态查询不自动初始化运动，也不会读取 Stream Motion 变量或运动限制。
显式调用 `initializeRemoteMotion()` 后才应发送运动指令；结束运动使用 `abort()`，
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
超时必须为有限值；NaN 和无穷大抛出 `std::invalid_argument`。
网络断开、超时或控制器返回非零 `ErrorID` 时抛出异常，调用者应处理错误。
`check*` 无对应推送包时返回 `std::nullopt`，不代表通信失败。
扩展命令是否可用取决于控制器版本与选项，尤其是变量写入。

示例不发送运动或 I/O 写命令：

```bash
./build/fanuc_driver_example 192.168.1.100
```

## 已实现功能

| 功能 | `Fanuc_driver` 方法 |
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
程序调用超过 10 个参数时在发送前抛出 `std::invalid_argument`，不再静默丢弃多余参数。
通过 `getLastInstructionResponse()` 主动接收并检查最新响应的 `SequenceID` 与 `ErrorID`。
`getRemainingBufferSize()` 按 8 条指令窗口计算，不会自动限流或阻止发送；
初始化会重置序列号。关节角采用控制器原生的度，位置采用毫米，
速度单位由请求的 `SpeedType` 决定，不做弧度或耦合关节转换。

**运动和写入接口会改变真实控制器状态。请确认安全条件后使用，不要将通信 API 当作安全功能。**
