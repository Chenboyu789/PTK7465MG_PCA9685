# PTK7465MG_PCA9685

基于 **ESP32-S3 + PCA9685 + ESP-IDF 5.5.3** 的 16 路舵机运动控制项目，面向 PTK7465MG 的运动与后续负载测试。固件工程名为 `ServoTorqueTesting`，生成的固件和部分日志沿用该名称。

默认启动后，16 路舵机以五次多项式 S 曲线同步往返，端点保持期间持续输出 PWM。项目目前实现脉宽控制，**尚未实现角度反馈、扭矩测量或电流采集**。

[快速启动](#快速启动) · [接线表](#硬件与接线) · [默认运动参数](#默认运动参数) · [常见问题](#常见问题) · [详细项目说明](项目说明.md)

## 功能与验证状态

| 项目 | 当前状态 |
| --- | --- |
| 16 路脉宽输出、逐通道范围校验 | 已实现 |
| 五次多项式时间缩放、往返与端点保持 | 已实现 |
| 单任务控制、异步业务请求、状态查询与故障清理重试 | 已实现 |
| 驱动和应用主机测试 | 2026-10-20 本地测试通过 |
| ESP32-S3 固件编译、链接、镜像与分区大小检查 | 2026-10-20 本地构建通过 |
| 最新固件烧录、真实 PWM 波形及舵机负载验证 | 尚未记录验证结果 |
| CI、多平台主机测试 | 尚未配置；现有主机测试脚本使用 Windows/MSVC |

主机测试使用模拟 I2C、时钟和 RTOS 接口，不证明真实波形、机械到位或扭矩性能。PTK7465MG 参数表标注刷新率为 333 Hz，未给出可接受的频率范围；项目的 321.24 Hz 配置仍需硬件验证。

## 硬件与接线

准备 ESP32-S3 开发板、PCA9685 舵机模块、PTK7465MG 舵机、合适的外部舵机电源、数据线与连接线。当前 Flash 配置为 **16 MB / QIO / 80 MHz**，应与实际模组匹配；GPIO38、GPIO39 需要在所用开发板上可用且无板载功能冲突。

下表面向具有独立逻辑电源 `VCC` 和舵机电源 `V+` 的 PCA9685 模块。不同模块的排针布局、上拉和供电连接可能不同，应按实际丝印或原理图核对，不能仅按线色判断。

| 连接来源 | 连接目标 | 说明 |
| --- | --- | --- |
| ESP32-S3 GPIO39 | PCA9685 SDA | I2C 数据线 |
| ESP32-S3 GPIO38 | PCA9685 SCL | I2C 时钟线 |
| ESP32-S3 3.3 V | PCA9685 VCC | 逻辑电源；本项目按 3.3 V 逻辑接线 |
| ESP32-S3 GND | PCA9685 GND | 控制器与模块共地 |
| 外部舵机电源正极 | 模块舵机电源 V+ | 电压应同时符合舵机与模块额定范围 |
| 外部舵机电源负极 | PCA9685 GND | 与 ESP32-S3 GND 相连 |
| PCA9685 CH0～CH15 的信号端 | 对应舵机信号线 | 通道编号从 0 开始 |
| 对应通道的 V+ / GND | 舵机电源正极 / 地线 | 使用模块电源排针前确认其连通关系和承载能力 |
| PCA9685 OE | 低电平，按模块要求接 GND | 当前代码不控制 OE；确认现有下拉或接线 |
| PCA9685 地址引脚 | 按模块说明设为 0x40 | 与固件的 7 位 I2C 地址一致 |

I2C 使用接至 **3.3 V** 的合适外部上拉，先核对模块现有上拉。不要将舵机电源 V+ 接到 ESP32-S3 的 3.3 V 或 PCA9685 的逻辑 VCC。PCA9685 的 VCC 和 OE 电气定义见 [NXP 数据手册](https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf)。

**烧录后会自动开始运动。** 首次上电建议只连接一个空载舵机，确认供电、方向和行程，再逐步增加通道和负载。初次 600 μs 定位不经过从实际位置出发的插值。需要立即停止机械动作时应有可操作的舵机电源断开方式；当前串口只输出日志，没有交互式停止命令。

## 快速启动

### 1. 准备开发环境

安装 Git 和 **ESP-IDF 5.5.3** 及其 ESP32-S3 工具链。环境安装与激活参见 [Espressif 官方指南](https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32s3/get-started/index.html)。在已激活的 ESP-IDF 终端中确认：

```powershell
idf.py --version
```

应显示 `ESP-IDF v5.5.3`。固件构建已在 Windows 本地环境验证，其他平台尚未记录验证结果。`.devcontainer` 的镜像标签默认使用 `latest`，不代表已固定或验证为 5.5.3；复现时优先使用明确的 5.5.3 环境。

### 2. 克隆并编译

```powershell
git clone https://github.com/Chenboyu789/PTK7465MG_PCA9685.git
cd PTK7465MG_PCA9685
idf.py set-target esp32s3
idf.py build
```

`set-target` 用于首次配置；已有本地配置时，重新设置 target 可能重建配置，修改前保存需要保留的设置。默认值来自 `sdkconfig.defaults`，本地产生的 `sdkconfig` 和 `build/` 不提交到仓库。

编译成功后生成 `build/ServoTorqueTesting.bin` 等文件。首次克隆应使用完整的 `idf.py build` 流程；不要依赖其他机器遗留的构建目录。

### 3. 烧录并查看日志

确认接线与供电后执行：

```powershell
idf.py -p COM9 flash monitor
```

将 `COM9` 换为开发板实际串口。控制台为 **UART0、115200 波特率**，USB 接口是否连接 UART0 取决于开发板电路。使用 `Ctrl+]` 退出监视器；退出监视器不会停止固件或舵机运动。

成功启动时，日志中应依次包含以下消息；时间戳、其他初始化日志可能不同。这是根据代码列出的预期消息，不是硬件实测记录：

```text
app: ServoTorqueTesting started
app: PCA9685 ready; all channels OFF
app: 16-channel motion started (servo cycle)
```

第一条表示进入应用初始化；最后一条表示循环业务启动成功，不代表机械已经到位。

## 默认运动参数

| 通道 | 往返脉宽 |
| --- | --- |
| CH0、CH4、CH8、CH12 | 600 ↔ 1000 μs |
| CH1、CH5、CH9、CH13 | 600 ↔ 1500 μs |
| CH2、CH6、CH10、CH14 | 600 ↔ 2000 μs |
| CH3、CH7、CH11、CH15 | 600 ↔ 2500 μs |

- PWM 请求频率为 **321.24 Hz**；按内部标称 25 MHz 和 `PRE_SCALE=18` 计算，理论频率为 **321.2377 Hz**，周期为 **3112.96 μs**，实际值需测量。
- 启动输出 600 μs 并保持 2 秒；单向运动 2 秒，每个端点保持 2 秒，持续循环。
- 运动命令约每 20 ms 更新，应用任务约每 5 ms 检查；保持期间 PCA9685 持续生成最后设定的 PWM。
- 脉宽未标定为实际角度。不同通道行程不同，但共享运动时间。

| 要修改的内容 | 入口 |
| --- | --- |
| I2C 引脚、地址、PWM 频率与标称时钟 | [board_config.h](components/board/include/board_config.h) |
| 运动起点、终点、允许范围及更新间隔 | [app_profile.c](components/application/app_profile.c) |
| 单向运动和保持时间 | `idf.py menuconfig` → `Servo motion test` |
| 算法、分层及应用 API | [项目说明](项目说明.md) |

已有 `sdkconfig` 中的值优先于 `sdkconfig.defaults`；只改 defaults 不会覆盖现有配置。修改运动端点时保证其位于允许范围内，并更新测试与文档。

## 测试

在安装 MSVC C/C++ 工具的 **Visual Studio Developer PowerShell** 中，进入仓库根目录执行：

```powershell
./tests/pca9685/run_host_tests.ps1
./tests/servo/run_host_tests.ps1
```

通过时各自输出以 `PASS:` 开头的结果。覆盖驱动初始化、脉宽换算、边界和故障重试，以及运动轨迹、保持时序、业务切换、状态查询和启动清理。详细范围与构建记录见[项目说明的验证部分](项目说明.md#验证记录与复现边界)。

`pytest_hello_world.py` 是需要 pytest-embedded 环境和真实开发板、PCA9685 的启动冒烟测试，执行时会启动运动业务；不属于上述无硬件主机测试，也尚未记录通过结果。

## 常见问题

| 现象 | 排查方向 |
| --- | --- |
| 找不到 `idf.py`、Python 依赖或工具链 | 使用已激活的 ESP-IDF 5.5.3 终端；按官方安装流程补齐对应版本工具，避免混用 Python 环境 |
| 提示缺少 `espidf.constraints.v5.5.txt` | 检查 `IDF_TOOLS_PATH` 是否指向完整的对应安装，修复安装并重新激活；不要将其他机器的现有 CMake 缓存作为通用修复 |
| 无法烧录、串口打不开 | 核对数据线、串口和下载模式，关闭占用串口的监视器；按开发板说明操作 BOOT/RESET |
| 能烧录但看不到应用日志 | 检查 UART0 对应接口和 115200 波特率；原生 USB 与板载 USB-UART 不一定是同一路控制台 |
| `Application startup failed` 或 PCA9685 初始化错误 | 检查 VCC、共地、SDA/SCL、3.3 V 上拉及 0x40 地址；保留错误码判断是超时还是配置失败 |
| `EXTCLK is set` | 当前驱动仅使用内部时钟；检查是否曾配置外部时钟，按日志建议给 PCA9685 逻辑 VCC 断电重启后再试 |
| 有启动日志但舵机不动 | 检查独立 V+ 供电、信号排针方向、OE 是否为低、舵机型号和波形；最后一条日志不证明舵机已响应 |
| 舵机抖动、啸叫或主控重启 | 先减少通道和负载，检查电源压降、地线及机械限位；用波形和供电测量区分原因 |
| 到端点后仍有 PWM 或保持力 | 当前设计如此：保持阶段不关断 PWM，仅停止更新脉宽 |
| 如何停止运动 | 当前没有串口 CLI；应用代码通过 `app_select_business(APP_BUSINESS_NONE)` 请求关闭全部输出，硬件紧急停止使用电源断开方式 |
| 主机测试找不到 `cl.exe` | 在 Visual Studio Developer PowerShell 中运行，并安装 C/C++ 工具；普通 ESP-IDF 终端不一定包含 MSVC |

更详细的错误状态和恢复语义见[项目说明](项目说明.md)。

## 代码与文档导航

```text
main/                     应用启动入口
components/application/   配置组装、队列、调度器和运动业务
components/servo/         舵机允许范围与输出服务
components/board/         板级 I2C 和硬件资源管理
components/pca9685/       PCA9685 驱动与电气脉宽转换
tests/                    无硬件主机测试及接口替身
```

[项目说明](项目说明.md) 介绍分层、状态机、算法、API、资源生命周期、舵机参数来源和 Flash 分区。[NXP 数据手册](https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf) 与 [ESP-IDF I2C 文档](https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32s3/api-reference/peripherals/i2c.html) 提供芯片和平台接口依据。

## 问题反馈与贡献

在 [GitHub Issues](https://github.com/Chenboyu789/PTK7465MG_PCA9685/issues) 提供提交号、ESP-IDF 版本、开发板与 Flash 规格、PCA9685 模块、舵机与供电、接线、完整错误日志、复现步骤和预期行为。波形相关问题附测量条件与结果，避免仅凭软件理论值判断实际输出。

修改代码时保持分层与单任务输出所有权，运行相关主机测试及固件编译，并同步参数和接口文档。仓库目前没有 CI，提交记录也不等同于硬件验证记录。

## 许可状态

仓库目前尚未指定许可证，未提供 `LICENSE` 文件。许可选择由维护者决定；本文不声明项目采用 MIT、Apache-2.0 等许可证。ESP-IDF 和其他第三方资料的许可按各自项目说明处理。
