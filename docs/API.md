# FANUC RMI C++ 接口参考

本文档对应 `fanuc_driver` 3.x 的公开 C++23 API，依据 `include/fanuc_driver/` 中的头文件编写。唯一的连接与命令入口是全局作用域的 `Fanuc_driver` 类；RMI 请求、响应和运动数据结构位于 `rmi` 命名空间。

```cpp
#include <fanuc_driver/fanuc_driver.hpp>
```

## 1. 编译和链接

通过 CMake 安装包使用：

```cmake
find_package(fanuc_driver 3 REQUIRED)
target_link_libraries(my_app PRIVATE fanuc_driver::fanuc_driver)
```

项目要求支持 `std::expected` 的 C++23 编译器与标准库，构建时需要 Boost 1.74+
开发头文件。TCP 使用 `boost::asio`（C++23 尚无 `std::asio`），Boost 是私有实现依赖，
安装包使用者不需要在公开 API 中包含或链接 Boost。构建示例和测试的项目命令见仓库根目录的 README。

## 2. 连接和典型调用顺序

构造函数立即连接控制器 RMI 端口（默认 `16001`）；`connect()` 才执行 RMI 握手，并自动将 TCP 连接切换到控制器返回的会话端口。状态读取不要求初始化运动。只有发送运动/队列指令前才调用 `initializeRemoteMotion()`。

```cpp
#include <exception>
#include <iostream>

#include <fanuc_driver/fanuc_driver.hpp>

int main()
{
  try
  {
    Fanuc_driver driver("192.168.1.100");  // 默认端口 16001
    const auto connection = driver.connect();
    std::cout << "RMI " << connection.MajorVersion << '.'
              << connection.MinorVersion << '\n';

    const auto status = driver.getStatus();
    const auto joints = driver.readJointAngles();
    std::cout << "ServoReady=" << static_cast<int>(status.ServoReady)
              << ", J1=" << joints.JointAngle.J1 << " deg\n";

    driver.disconnect();
  }
  catch (const std::exception& e)
  {
    std::cerr << "RMI error: " << e.what() << '\n';
    return 1;
  }
}
```

`connect()` 返回 `rmi::ConnectPacket::Response`，其中包括协商到的 `PortNumber` 和控制器 RMI `MajorVersion` / `MinorVersion`。成功后可进行状态、读写或运动调用。`disconnect()` 结束 RMI 会话；析构函数只释放 TCP 资源，不发送停止、复位或断开命令。不要依赖析构函数作为机器人停止机制。

一个 `Fanuc_driver` 对象不可复制或移动，内部连接和指令序号由对象独立持有。虽然实现内部对部分共享状态加锁，同一实例仍建议只由一个应用线程调用；该库不提供轮询线程、运动控制循环或实时流接口。
Asio 异步操作在调用线程中由 `io_context` 驱动；同步接口等待网络事件而非忙轮询。
`check*` 只处理已就绪的数据，不等待新包。TCP 发送使用组合写操作处理短写，
接收支持 CRLF 半包与粘包；切换会话端口会清除旧连接未完成的帧。

## 3. 通用约定

### 超时与错误

- 具有 `timeout` 参数的方法以**秒**计时；省略参数或传入 `std::nullopt` 时等待 1 秒。
- 传入负值表示不设响应等待期限。超时只控制等待响应，不会取消已经发送给控制器的命令。
- 超时必须是有限值；NaN 和无穷大抛出 `std::invalid_argument`。超时后保留待接收数据，
  后续调用可继续收取响应。超时不限制 DNS 解析、TCP 建连或发送所花的时间。
- TCP 连接/读写失败、响应超时以及控制器响应中的非零 `ErrorID` 会以异常报告，通常为 `std::runtime_error`。写调用方应在应用边界捕获异常并决定如何处理。
- `getErrorMessageString(code)` 将已知 RMI 错误码转换为文本；未知值返回带错误码的说明。控制器产生的报警文本由 `readError()` 获取，与接口抛出的 `ErrorID` 异常不同。
- `readError(timeout, count)` 的 `count` 是可选的，协议响应中的有效范围为 1–5；具体控制器版本也可能限制可读报警数。

### 数值和可选参数

- 位置寄存器编号在本库中检查为 1–100；数值寄存器编号检查为 1–200。超出范围会在发送前抛出异常。
- `group`、`groupmask` 和可选运动字段用 `std::optional` 表示；省略时不在请求中写入该字段，并非设为数值零。
- 返回值是按值解析得到的 RMI 响应结构。状态字段通常保留控制器定义的整数/字节值，库不将它们转换为枚举。
- `std::variant<int, float>` 表示整数或浮点数。数字寄存器、通用 I/O 和系统变量的写接口只接受这两种数值类型，不接受字符串。

## 4. 功能接口索引

以下方法均为 `Fanuc_driver` 的公开成员。除特别注明外，可选 `timeout` 参数均遵循上述秒数规则。

### 会话、初始化和控制

| 方法 | 用途 / 关键参数 | 返回值 |
|---|---|---|
| `Fanuc_driver(const std::string& robot_ip_address, uint16_t rmi_port = 16001)` | 建立到控制器初始 RMI 端口的 TCP 连接 | 无；连接失败抛异常 |
| `connect(timeout = std::nullopt)` | 执行 `FRC_Connect` 握手，并连接协商的会话端口 | `ConnectPacket::Response` |
| `disconnect(timeout = std::nullopt)` | 发送 `FRC_Disconnect` | `DisconnectPacket::Response` |
| `initializeRemoteMotion(timeout = std::nullopt)` | 初始化 Remote Motion；重置本地运动序号和最近指令响应 | `InitializePacket::Response` |
| `initializeRemoteMotion(timeout, groupmask, rtsa, pltzmode)` | 初始化时附加可选 GroupMask、RTSA 和 PLTZMODE | `InitializePacket::Response` |
| `pause(timeout = std::nullopt)` / `resume(timeout = std::nullopt)` | 暂停 / 继续 TP 程序（`resume` 对应 `FRC_Continue`） | 对应响应结构 |
| `abort(timeout = std::nullopt)` | 中止 Remote Motion TP 程序 | `AbortPacket::Response` |
| `reset(timeout = std::nullopt)` | 请求复位机器人控制器 | `ResetRobotPacket::Response` |
| `setSpeedOverride(int value, timeout = std::nullopt)` | 设置控制器速度倍率 | `SetSpeedOverridePacket::Response` |

`initializeRemoteMotion()` 每次调用都会将运动序号重置为 1，并清除缓存的最近指令响应。只有控制器接受初始化后才发送运动指令。

### 状态、报警和位置读取

| 方法 | 用途 / 返回字段 |
|---|---|
| `getStatus(timeout)` | 控制器主要状态：`ServoReady`、`TPMode`、`RMIMotionStatus`、`ProgramStatus`、`SingleStepMode`、`NumberUTool`、`NextSequenceID`、`NumberUFrame`、`Override` |
| `getExtendedStatus(timeout)` | 扩展状态：`InMotion`、`DrivesPowered`、`GenOverride`，以及可选 `ErrorCode`、`ControlMode`、`SpeedClampLimit` |
| `readError(timeout)` / `readError(timeout, count)` | 读取报警文本；响应字段为 `ErrorData` 和可选的 `ErrorData2`–`ErrorData5` |
| `readJointAngles(group = nullopt, timeout = nullopt)` | 读取指定组关节角，响应数据为 `JointAngle` |
| `getCartesianPosition(timeout = nullopt, group = nullopt)` | 读取笛卡尔位置、构型和 `TimeTag` |
| `getTCPSpeed(timeout = nullopt, group = nullopt)` | 读取 TCP 速度和 `TimeTag` |

`readJointAngles` 的参数次序与 `getCartesianPosition` / `getTCPSpeed` 不同，调用时请显式传入参数名对应的位置；若需要指定超时和组，可写 `readJointAngles(group, timeout)`。

### 用户坐标系、工具坐标系和位置寄存器

| 方法 | 用途 / 关键参数 |
|---|---|
| `getUFrameUTool(timeout, group)` | 查询当前用户坐标系 `UFrameNumber` 和工具坐标系 `UToolNumber` |
| `setUFrameUTool(uframe, utool, timeout, group)` | 设置当前用户/工具坐标系编号 |
| `readUFrameData(uframe, timeout, group)` / `writeUFrameData(uframe, data, timeout, group)` | 读取 / 写入用户坐标系 `FrameData` |
| `readUToolData(utool, timeout, group)` / `writeUToolData(utool, data, timeout, group)` | 读取 / 写入工具坐标系 `FrameData` |
| `readPositionRegister(register_number, timeout)` | 读取位置寄存器；响应的 `Representation` 指明表示形式，`Configuration`、`Position`、`JointAngle` 以可选值返回 |
| `writePositionRegister(register_number, representation, configuration, position, joint_angle, timeout)` | 写入位置寄存器；`representation` 为 `"Joint"`（大小写不敏感）时使用 `joint_angle`，否则发送笛卡尔 `configuration` 和 `position` |

写坐标系数据和写位置寄存器会修改控制器配置/寄存器。`FrameData` 的字段是 `X/Y/Z/W/P/R`；`PositionData` 另外包含 `Ext1/Ext2/Ext3`。

### 寄存器、I/O 和系统变量

| 方法 | 用途 / 说明 |
|---|---|
| `readNumericRegister(register_number, timeout)` / `writeNumericRegister(register_number, value, timeout)` | 读取 / 写入数值寄存器（1–200）；写值为 `int` 或 `float` |
| `readDigitalInputPort(port_number, timeout)` | 读取数字输入；响应值为 `PortValue` |
| `writeDigitalOutputPort(port_number, port_value, timeout)` | 将数字输出写为 `true`（ON）或 `false`（OFF） |
| `readIOPort(port_type, port_number, timeout)` / `writeIOPort(port_number, port_type, port_value, timeout)` | 通用 I/O 读写；端口类型和可用范围由控制器配置决定，写值为整数或浮点数 |
| `readVariablePacket(variable_name, timeout)` / `writeVariablePacket(variable_name, value, timeout)` | 读取 / 写入数值型系统变量；名称按 RMI 约定包含 `$`，协议结构注明最多 64 字节；写值为整数或浮点数 |

通用 I/O 写入结构注明常见类型 `AO`、`GO`、`DO`、`RO`、`FLAG`；读写能力和有效类型依控制器及选件而异。变量读写尤其受控制器版本、变量类型与访问权限限制。写输出、寄存器或变量之前，请确认目标端口/变量及机器人现场状态。

### 负载表

| 方法 | 用途 / 单位 |
|---|---|
| `getPayloadSchedule(timeout)` / `setPayloadSchedule(schedule, timeout)` | 查询 / 选择当前负载表 |
| `getPayloadValue(schedule, timeout)` / `setPayloadValue(schedule, mass, cg_x, cg_y, cg_z, use_in, in_x, in_y, in_z, timeout)` | 查询 / 设置负载值 |
| `getPayloadComp(schedule, timeout)` / `setPayloadComp(schedule, mass, cg_x, cg_y, cg_z, in_x, in_y, in_z, timeout)` | 查询 / 设置负载补偿值 |

写入接口的质心 `cg_x/y/z` 输入单位为米，库在发送前转换为 RMI 的厘米；惯性 `in_x/y/z` 输入单位为 kg·m²，库转换为 kg·cm²。`mass` 不做单位转换，按控制器 RMI 单位传递。`setPayloadValue` 只有在 `use_in == true` 时才发送惯性字段；若需要设置补偿惯性，使用 `setPayloadComp`。负载修改会影响机器人运动，请先核对控制器配置和单位。

### TP 程序调用

| 方法 | 用途 / 返回 |
|---|---|
| `programCall(program_name, timeout = nullopt)` | 同步请求调用 TP 程序并等待同一 `SequenceID` 的响应 |
| `programCall(program_name, timeout, params)` | 同步调用并传递参数 |
| `programCallNonBlocking(program_name)` / `programCallNonBlocking(program_name, params)` | 发送调用请求后立即返回已分配序号的 `ProgramCallPacket::Request` |

参数类型为 `std::vector<rmi::RMICallParam>`；每个元素是 `std::pair<std::string, std::variant<int, float, std::string>>`，依次对应 RMI 的 `ParamTypeN` 和 `ParamValueN`。最多传 10 个参数，超过时在发送前抛出 `std::invalid_argument`，不再静默丢弃多余参数。

```cpp
const std::vector<rmi::RMICallParam> args{
  {"Integer", 7},
  {"String", std::string("part-A")},
  {"Float", 1.25f},
};
const auto result = driver.programCall("MY_PROGRAM", 2.0, args);
std::cout << "SequenceID=" << result.SequenceID << '\n';
```

程序名、类型标记及参数值必须匹配控制器上 TP 程序的定义。程序调用/参数传输会使用 `setEncoding()` 指定的编码。

### 运动与队列指令

#### 支持的指令类型

| 运动/队列功能 | `rmi::*Packet` 类型 |
|---|---|
| 关节绝对 / 相对运动（笛卡尔位置表示） | `JointMotionPacket` / `JointRelativePacket` |
| 关节绝对 / 相对运动（关节表示） | `JointMotionJRepPacket` / `JointRelativeJRepPacket` |
| 直线绝对 / 相对运动（笛卡尔位置表示） | `LinearMotionPacket` / `LinearRelativePacket` |
| 直线绝对 / 相对运动（关节表示） | `LinearMotionJRepPacket` / `LinearRelativeJRepPacket` |
| 圆弧绝对 / 相对运动 | `CircularMotionPacket` / `CircularRelativePacket` |
| 样条运动（笛卡尔 / 关节表示） | `SplineMotionPacket` / `SplineMotionJRepPacket` |
| 等待数字输入 / 等待时间 | `WaitForDINPacket` / `WaitForTimePacket` |
| 队列内切换用户坐标系 / 工具坐标系 / 负载表 | `SetUFramePacket` / `SetToolFramePacket` / `SetPayloadInstructionPacket` |

`sendRMIPacket<T>(request, timeout = nullopt)` 对支持的指令发送请求并等待相同序号的响应，返回 `T::Response`。`sendRMIPacketNonBlocking(request)` 只发送指令并将分配的 `SequenceID` 写回传入的请求对象；它不等待指令响应。`sendJointMotion(request, timeout)` 是 `JointMotionJRepPacket` 的同步便捷接口。同步模板的实例化类型必须是库显式支持的上述 packet 类型。

每种运动请求结构的核心字段如下：

- `Configuration` 和 `Position`：笛卡尔目标构型和位置；使用 `*JRepPacket` 时改用 `JointAngle`。
- 圆弧请求还需设置 `ViaConfiguration` 和 `ViaPosition`。
- `SpeedType`、`Speed`、`TermType`、`TermValue`：速度与终止类型/数值，由 RMI 和控制器配置约束。
- 可选字段按相应 packet 结构提供，例如 `ACC`、`OffsetPRNumber`、`VisionPRNumber`、`WristJoint`、`MROT`、`LCBType` / `LCBValue`、I/O 触发字段、`ToolOffsetPRNumber`、`ALIM` / `ALIMREG`、`NoBlend`。并非所有可选字段适用于每种运动类型；具体成员以对应 `Request` 声明为准。

```cpp
// 仅展示请求构造方式；发送前必须独立完成安全审查。
// 此示例实际发送运动命令，会改变真实机器人状态。
rmi::LinearRelativePacket::Request move{};
move.Position.Z = 10.0f;
move.SpeedType = "mmSec";
move.Speed = 10;
move.TermType = "FINE";
move.TermValue = 0;
const auto response = driver.sendRMIPacket<rmi::LinearRelativePacket>(move);
```

上例的数字只是结构赋值示例，不是速度、距离或安全建议。使用前必须按对应控制器的 RMI 手册、工具/用户坐标系、有效范围和现场安全程序校验全部字段。关节角使用控制器原生角度（度），笛卡尔位置为毫米；速度值不在库中换算，其含义由 `SpeedType` 决定。

序号从 1 开始，初始化时重置。异步发送后，用 `getLastInstructionResponse()` 主动取回当前收到的最新异步指令响应，并检查其 `SequenceID` 与 `ErrorID`。不应把“已经发送”当作“执行成功”。

控制器指令窗口按 8 条计算。`getRemainingBufferSize()` 返回估算的剩余窗口数，但**不会等待确认、限流、阻止超量发送或为应用做队列调度**。调用方需自行检查响应并安排发送节奏。

### 推送包、诊断和编码

| 方法 | 用途 / 语义 |
|---|---|
| `getLastInstructionResponse()` | 获取最新收到的指令响应；无响应时返回空 `std::optional` |
| `getRemainingBufferSize()` | 查询估算剩余的指令窗口容量（窗口大小为 8） |
| `checkSystemFault()` | 收取匹配的 `SystemFaultPacket`，包含关联的 `SequenceID` |
| `checkTimeoutTerminate()` | 收取控制器的超时终止通知 |
| `checkCommunicationPacket()` | 收取通信通知（如 `FRC_AsbnReady`） |
| `checkUnknownPacket()` | 收取未识别命令及其错误码 |
| `getErrorMessageString(uint32_t error_code)` | 将 RMI 错误码转为说明文本 |
| `setEncoding(const std::string& encoding)` | 设置程序调用字符串的输出编码及报警文本解码编码；默认 `UTF-8` |

`check*` 方法没有对应推送包时返回 `std::nullopt`，这不是通信错误。它们会消费已匹配到的推送包；如果一次读取到多条同类包，会返回最后一条。建议由一个明确的接收/处理逻辑负责调用，避免多个线程竞争读取同一实例的推送消息。`setEncoding("SHIFT-JIS")` 可用于需要 SHIFT-JIS 的控制器报警文本和程序调用字符串；库默认使用 UTF-8。

## 5. RMI 数据结构

类型定义位于 `<fanuc_driver/packets.hpp>`，包含该头文件即可通过主头文件使用。结构字段采用与 RMI JSON 键一致的 PascalCase，名称是协议序列化的一部分，不应自行改名。

### 常用位置类型

| 类型 | 字段 | 说明 |
|---|---|---|
| `rmi::FrameData` | `X, Y, Z, W, P, R` | 用户坐标系 / 工具坐标系数据 |
| `rmi::PositionData` | `X, Y, Z, W, P, R, Ext1, Ext2, Ext3` | 笛卡尔位置及外部轴 |
| `rmi::ConfigurationData` | `UToolNumber, UFrameNumber, Front, Up, Left, Flip, Turn4, Turn5, Turn6` | 位置构型 |
| `rmi::JointAngleData` | `J1`–`J9` | 关节角度；未用关节通常保持为 0 |
| `rmi::CartesianMotionGroup` | `Configuration, Position` | 构型与笛卡尔位置组合 |
| `rmi::InstructionResponse` | `Instruction, ErrorID, SequenceID` | 最近指令响应的通用视图 |
| `rmi::RMICallParam` | `pair<string, variant<int,float,string>>` | TP 程序调用参数 |

### Packet 类型的 Request / Response

协议结构按功能命名为 `rmi::<Packet>`，其嵌套类型 `Request` 和 `Response` 对应 JSON 请求与响应。例如，`rmi::StatusRequestPacket::Response` 提供状态字段；`rmi::LinearMotionPacket::Request` 描述线性运动请求。上方功能表已列出本库 `Fanuc_driver` 支持发送的命令和运动类型。协议头文件还定义了部分解析用结构（如 `UnknownPacket`、`CommunicationPacket`、`TimeoutTerminatePacket`、`SystemFaultPacket`），由相应 `check*` 方法暴露。

## 6. 资源、线程和安全边界

- 构造函数、连接和会话操作可能因 IP、端口、网络、防火墙或控制器 RMI 配置失败。确保控制器已启用相应 RMI 功能并允许主机访问。
- 本接口不提供安全停机、碰撞检测、运动规划、安全区域、I/O 联锁或机器人状态机封装。`abort()`、`pause()` 和 `reset()` 是普通 RMI 命令，不是认证安全功能。
- 任一运动、输出写入、寄存器/变量修改、坐标系或负载写入都可能改变真实设备或程序行为。先在安全的控制器配置中验证，并遵循 FANUC 控制器手册及现场安全流程。
- `Fanuc_driver` 是会话级对象；保持对象生命周期覆盖所有命令，完成后显式 `disconnect()`。网络断开后，本库不承诺自动重连或自动恢复会话/序号。
