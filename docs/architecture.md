# 目录与依赖边界

项目采用契约层 `core` 和品牌适配层 `drivers/<brand>`，测试目录与实现层对应。
公开的 `Fanuc_driver` 通过 PImpl 复用内部 RMI 驱动，仅暴露 `Robot_driver` 的契约，
不引入示意架构中尚未实现的工厂、型号、运动学、轨迹规划或其他品牌协议。

## 公共头文件

- `include/robot_driver/core/robot.hpp`：品牌无关的抽象驱动契约。
- `include/robot_driver/core/types.hpp`：品牌无关的数据类型 `RobotStatus`。
- `include/robot_driver/core/error.hpp`：通用状态、错误码和响应类型。
- `include/robot_driver/drivers/fanuc/robot.hpp`：只依赖标准库和 `core` 的 FANUC 驱动入口。
- `include/robot_driver/robot_driver.hpp`：聚合上述公共接口，适合需要具体 FANUC 驱动的应用。

`core` 只能依赖标准库和本层头文件，不能包含品牌头文件或 `src` 中的内容。
品牌公共头文件可以依赖 `core`，但不能包含内部实现头文件。
只依赖通用接口的业务代码应使用 `core/robot.hpp`，而不是聚合入口。

公开控制接口只返回 `ResponseStatus`，查询使用 `RobotStatus` 和 `bool`。
公共头文件不包含或声明 `rmi` 类型；`ResponseStatus` 不包含原始控制器码字段。
RMI 非零 `ErrorID` 映射为统一自定义码 9999，说明写入 `detail`。

## 内部实现

`src/core/error.cpp` 实现通用状态消息。
`src/drivers/fanuc/robot.cpp` 实现公开驱动的 PImpl 和通用接口适配；
`rmi_driver.cpp` 保留 TCP/RMI 会话和原有扩展协议实现。
`detail/` 保存 `packets.hpp`、`types.hpp`、`rmi_driver.hpp` 和
JSON 映射、分帧、序列化声明及错误适配头文件。
内部驱动 `FanucRmiDriver` 仍可使用协议报文，公开类不提供报文发送、寄存器、
运动、推送或扩展参数接口；这些能力不因移动头文件而伪装成通用 API。
FANUC 实现使用 RMI，不依赖或模拟 FOCAS。

### 同步命令与连续异步运动

内部 `FanucRmiDriver::sendInstructionAsync(request)` 接收带 `Instruction`、
`SequenceID` 的请求，复制请求并返回 `InstructionHandle`，包含分配的 `sequence_id`
和 `std::shared_future<Response<rmi::InstructionResponse>>`。调用不等待网络写入或
控制器回复；每条结果独立保存，后续成功不会覆盖前面的错误，也不需要轮询驱动来推进接收。
这仍是内部协议接口，不改变安装后的 `Fanuc_driver` 或 `Robot_driver` 契约。

```cpp
auto first = driver.sendInstructionAsync(first_request);
auto second = driver.sendInstructionAsync(second_request);

do_other_work();

if (first.result.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
{
  const auto& result = first.result.get();
  // 检查 result.status；控制器回复存在时，data 保留原始 ErrorID 和 SequenceID。
}
const auto& second_result = second.result.get();
```

后台单线程持续运行 `io_context`，串行操作 socket、写队列和待回复表，因此无需
额外的 `strand`；不可直接从其他线程操作这些状态。一个接收协程和一个发送协程
持续运行，仅允许一个 `async_write` 在途，缓冲区由写操作持有直到协程恢复；
读循环持续分帧，按 `SequenceID` 分发运动回复。
沿用现有 RMI 的 8 条在途窗口，超出窗口的指令在本地排队，收到对应回复后再发送。
本地队列没有容量上限，长轨迹应由调用者分批提交，不能无限制生产。
`getRemainingBufferSize()` 计入尚未收到回复的在途和本地排队指令，最小返回 0。
普通同步命令可绕过被窗口阻塞的运动，避免状态查询或 Abort 等控制操作饿死。
这些命令可能先于尚未发送的运动到达控制器，因此同步查询不是运动完成屏障。

`sendJointMotion()`、带序号的 `sendRMIPacket()` 和同步 `programCall()` 复用
上述提交与结果等待机制，等待期间不占用运动提交锁。无序号的同步命令使用独立命令锁
串行执行完整请求/回复事务，仍可与多个异步运动交错。
旧的 `sendRMIPacketNonBlocking()` 保留兼容行为：等待写入但不等待回复，
窗口满时也可能等待空位；需要真正不等待的调用应迁移到 `sendInstructionAsync()`。
`getLastInstructionResponse()` 仅适合进度显示，不用于判断整批成功；整批成功必须检查
每个句柄的 `status.ok()`。

future 就绪表示收到该条 RMI 回复或确定无法继续追踪，不额外承诺机器人已精确到位。
应按控制器版本和轨迹终止模式解释 RMI 回复，不能把 TCP 写入完成视为运动完成。
`wait_for()` 超时只停止调用者等待，不取消运动，也不触发自动重发。
断线、销毁和会话替换会以连接错误结束未完成句柄；系统故障、Terminate 和成功的
Abort 会结束受影响任务并阻止后续指令，需成功初始化后才能重新提交。
已完成的句柄结果不会因此改变。初始化会拒绝仍有未完成指令的会话，
成功后才重置序号；旧句柄仍保留原结果。
调用者必须保证驱动生命周期覆盖所有成员函数调用，不能与析构并发使用驱动。
普通同步命令超时后，迟到回复仍可能到达；无序号协议不能可靠区分同类型重试，
应恢复会话后再重试，而不是把超时当作取消。

### Asio 协程设计

参考 Boost.Asio 官方 [C++20 examples](https://www.boost.org/doc/libs/latest/doc/html/boost_asio/examples/cpp20_examples.html)
中的协程 echo server、chat server 和超时示例，采用其结构化异步流程，但不照搬服务器协议：

- DNS 解析和 TCP 建连使用 `awaitable<ResponseStatus>`；同步构造和握手入口通过
  `co_spawn(..., use_future)` 驱动并取得结果，不改变公开接口或响应超时的范围。
- 接收和发送使用 `co_await` 循环代替递归回调，`as_tuple(use_awaitable)` 明确处理
  网络错误。两个 `co_spawn` 均有完成处理器，非预期的标准异常报告为 `SERVER_ERR`，
  不使用 `detached` 丢弃错误。
- 发送协程在没有可发送报文时等待 `steady_timer` 的最大时间点；入队或收到回复
  时用 `cancel_one()` 唤醒并重新检查队列。这沿用 chat server 的通知模式，
  不是定时轮询。重新检查 RMI 指令窗口，普通命令仍可绕过阻塞的运动指令。
- 接收缓冲和 CRLF 分帧器由接收协程持有。每次协商新会话前关闭旧 socket、
  唤醒发送协程并等待工作线程排空，旧半帧和协程不会进入新会话。
- 连接失败统一关闭 socket、唤醒等待者，并完成本地排队和在途写操作的 promise。
  写操作的完成是幂等的；错误或 Abort 与写完成交错时不会重复兑现，
  写缓冲仍保留到异步写真正结束。仅指令故障不会关闭仍可用于 Abort/Initialize 的 TCP 会话。

最低 Boost 版本提高到 1.83，以使用正式命名空间下的 `as_tuple`。
不引入实验性 channel / parallel group：现有单线程执行器和需要跳过满窗口指令的
队列已满足串行写入要求，没有额外的多线程互斥需求。
不把 `cancel_after` 或 watchdog 直接绑定到共享接收/运动发送：
响应等待期限到期只结束调用者的等待，后台仍要匹配迟到回复；
自动取消、重发运动或把 TCP 写入完成当成机器人到位会改变 RMI 语义。
本地队列容量限制以及无序号命令超时后的可靠重试仍需单独设计，
本次不引入新的丢弃策略或自动重连。

仅 `.cpp` 和针对内部行为的测试可以包含 `src` 下的头文件。
内部包含路径只对测试目标以 `PRIVATE` 设置；安装规则只复制 `include/`，
不发布 `src` 或内部 `detail`。不在 `include` 和 `src` 中重复存放同一份内部头文件。

## 构建与测试

根 `CMakeLists.txt` 负责库、依赖和安装导出，按选项进入 `tests/` 与 `examples/`。
这两个目录各自维护目标源文件清单；可执行文件仍生成在构建根目录，
因此原有 VS Code 调试配置和命令不变。
`tests/core/` 不依赖 FANUC，FANUC 内部适配器测试放在 `tests/drivers/fanuc/`。
`tests/consumer/` 通过安装后的 CMake 包编译聚合入口示例及独立契约、FANUC 消费者，
检查公开返回类型和协议头文件不被安装。

构建包仍为 `fanuc_driver`，导出目标仍为 `fanuc_driver::fanuc_driver`，
许可证继续保留于 `LICENSES/`。目录调整不意味着已实现独立的核心库构建目标。

## 头文件迁移

| 旧入口 | 新入口 |
|---|---|
| `robot_driver/robot_driver.hpp`（原纯契约入口） | `robot_driver/core/robot.hpp` |
| `robot_driver/response_status.hpp` | `robot_driver/core/error.hpp` |
| `fanuc_driver/fanuc_driver.hpp` | `robot_driver/drivers/fanuc/robot.hpp` |
| `robot_driver/drivers/fanuc/packets.hpp` | `src/drivers/fanuc/detail/packets.hpp`（仅内部） |
| `robot_driver/drivers/fanuc/types.hpp` | `src/drivers/fanuc/detail/types.hpp`（仅内部） |

`robot_driver/robot_driver.hpp` 路径现用作聚合入口；其他旧路径不提供转发兼容头。
迁移时修改包含路径并重新编译，建议安装到干净的前缀，避免旧安装遗留的头文件混用。
