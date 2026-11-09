# 目录与依赖边界

项目采用契约层 `core` 和品牌适配层 `drivers/<brand>`，测试目录与实现层对应。
本次仅重排现有功能，保留 `Robot_driver`、`Fanuc_driver`、`rmi` 的名称和原有方法语义，
不引入示意架构中尚未实现的工厂、型号、运动学、轨迹规划或其他品牌协议。

## 公共头文件

- `include/robot_driver/core/robot.hpp`：品牌无关的抽象驱动契约。
- `include/robot_driver/core/types.hpp`：品牌无关的数据类型 `RobotStatus`。
- `include/robot_driver/core/error.hpp`：通用状态、错误码和响应类型。
- `include/robot_driver/drivers/fanuc/robot.hpp`：FANUC RMI 驱动入口。
- `include/robot_driver/drivers/fanuc/packets.hpp`、`types.hpp`：公共方法参数和返回值需要的协议类型。
- `include/robot_driver/robot_driver.hpp`：聚合上述公共接口，适合需要具体 FANUC 驱动的应用。

`core` 只能依赖标准库和本层头文件，不能包含品牌头文件或 `src` 中的内容。
品牌公共头文件可以依赖 `core`，但不能包含内部实现头文件。
只依赖通用接口的业务代码应使用 `core/robot.hpp`，而不是聚合入口。

`packets.hpp` 和品牌 `types.hpp` 仍属于公开接口：当前请求参数、响应数据和异步通知会暴露其中类型。
不能仅移动到 `src` 而继续保留相同的公共接口；真正隐藏这些类型需要另行设计协议无关的数据模型。

## 内部实现

`src/core/error.cpp` 实现通用状态消息。
`src/drivers/fanuc/` 包含 TCP/RMI 会话和协议序列化实现；
其 `detail/` 保存 JSON 映射、分帧、序列化声明和错误适配头文件。
FANUC 实现使用 RMI，不依赖或模拟 FOCAS。

仅 `.cpp` 和针对内部行为的测试可以包含 `src` 下的头文件。
内部包含路径只对测试目标以 `PRIVATE` 设置；安装规则只复制 `include/`，
不发布 `src` 或内部 `detail`。不在 `include` 和 `src` 中重复存放同一份内部头文件。

## 构建与测试

根 `CMakeLists.txt` 负责库、依赖和安装导出，按选项进入 `tests/` 与 `examples/`。
这两个目录各自维护目标源文件清单；可执行文件仍生成在构建根目录，
因此原有 VS Code 调试配置和命令不变。
`tests/core/` 不依赖 FANUC，FANUC 内部适配器测试放在 `tests/drivers/fanuc/`。
`tests/consumer/` 通过安装后的 CMake 包编译聚合入口示例及纯契约消费者。

构建包仍为 `fanuc_driver`，导出目标仍为 `fanuc_driver::fanuc_driver`，
许可证继续保留于 `LICENSES/`。目录调整不意味着已实现独立的核心库构建目标。

## 头文件迁移

| 旧入口 | 新入口 |
|---|---|
| `robot_driver/robot_driver.hpp`（原纯契约入口） | `robot_driver/core/robot.hpp` |
| `robot_driver/response_status.hpp` | `robot_driver/core/error.hpp` |
| `fanuc_driver/fanuc_driver.hpp` | `robot_driver/drivers/fanuc/robot.hpp` |
| `fanuc_driver/packets.hpp` | `robot_driver/drivers/fanuc/packets.hpp` |
| `fanuc_driver/types.hpp` | `robot_driver/drivers/fanuc/types.hpp` |

`robot_driver/robot_driver.hpp` 路径现用作聚合入口；其他旧路径不提供转发兼容头。
迁移时修改包含路径并重新编译，建议安装到干净的前缀，避免旧安装遗留的头文件混用。
