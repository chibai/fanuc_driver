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
