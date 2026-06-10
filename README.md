# 工业级下位机代码框架规范

本文档用于指导工业级下位机工程的创建、分层、代码管理和发布维护。适用场景包括 MCU、RTOS、工业控制、电机控制、传感器采集、通信网关、执行器控制、边缘设备等长期维护型嵌入式产品。

目标不是堆叠目录，而是建立一套可移植、可测试、可追踪、可协作的工程组织方式。

---

## 1. 项目目标

一个优秀的下位机工程应满足以下目标：

- **硬件可替换**：业务代码不直接依赖芯片寄存器、HAL、LL 或厂商 SDK。
- **业务可维护**：产品功能按模块划分，职责清晰，接口稳定。
- **问题可追踪**：日志、错误码、故障码、版本号、构建信息完整。
- **质量可验证**：核心算法、协议解析、参数管理等模块可以单元测试。
- **发布可复现**：固件产物、版本信息、配置项、提交记录可追溯。
- **团队可协作**：目录结构、命名规范、提交规范、分支流程统一。

---

## 2. 设计原则

### 2.1 分层隔离

工程依赖方向应保持单向：

```text
app
 ↓
service
 ↓
middleware
 ↓
driver / platform
 ↓
hal / sdk / hardware
```

上层可以调用下层，下层不要反向依赖上层。底层需要通知上层时，使用回调、事件、消息队列或发布订阅机制。

### 2.2 硬件抽象

业务模块不应直接调用如下接口：

```c
HAL_UART_Transmit(...);
GPIO_SetBits(...);
TIM_SetCompare(...);
```

应通过 BSP、设备驱动或服务接口间接访问：

```c
Bsp_UartSend(UART_PORT_DEBUG, data, len);
Motor_SetTargetSpeed(speed);
Sensor_ReadTemperature(&temperature);
```

### 2.3 配置集中

所有板级配置、功能开关、通信参数、资源数量和产品差异应集中在 `config/` 或 `board/` 中管理，避免魔法数字散落在业务代码中。

### 2.4 接口稳定，内部可变

模块对外只暴露必要接口。内部结构、缓存、状态变量应使用 `static` 限制作用域，避免外部直接访问。

### 2.5 可测试优先

以下模块应尽量不依赖硬件，方便在 PC 或 CI 环境运行测试：

- 协议编解码
- CRC / checksum
- PID / 滤波 / 控制算法
- 参数校验
- 状态机
- 故障判定逻辑
- 环形缓冲区

---

## 3. 推荐目录结构

```text
project/
├── app/                         # 应用层：任务编排、产品流程、主业务入口
│   ├── app_main.c
│   ├── app_init.c
│   ├── task_comm.c
│   ├── task_control.c
│   └── task_monitor.c
│
├── service/                     # 服务层：面向产品功能的业务模块
│   ├── comm/                    # 通信服务：上位机、CAN、RS485、以太网等
│   ├── motor/                   # 电机控制服务
│   ├── sensor/                  # 传感器管理服务
│   ├── parameter/               # 参数管理服务
│   ├── fault/                   # 故障管理服务
│   ├── storage/                 # 数据存储服务
│   ├── upgrade/                 # OTA / IAP 升级服务
│   └── watchdog/                # 看门狗喂狗策略和任务健康监控
│
├── middleware/                  # 中间件：通用、可复用、弱业务相关能力
│   ├── protocol/                # 协议编解码、帧解析、粘包拆包
│   ├── crc/                     # CRC8 / CRC16 / CRC32
│   ├── ring_buffer/             # 环形缓冲区
│   ├── pid/                     # PID 控制器
│   ├── filter/                  # 均值滤波、低通滤波、卡尔曼滤波等
│   ├── fsm/                     # 通用有限状态机
│   ├── event/                   # 事件分发、发布订阅
│   └── log/                     # 日志格式化、等级控制、输出适配
│
├── driver/                      # 驱动层：BSP 和外部器件驱动
│   ├── bsp/                     # 板级外设抽象：UART、CAN、GPIO、ADC、PWM 等
│   ├── device/                  # 外部芯片驱动：IMU、Flash、EEPROM、编码器等
│   └── hal_port/                # 厂商 HAL / LL / SDK 的适配封装
│
├── platform/                    # 平台层：RTOS、编译器、芯片公共适配
│   ├── os/                      # FreeRTOS / RT-Thread / 裸机适配
│   ├── time/                    # tick、延时、时间戳
│   ├── critical/                # 临界区、锁、中断开关
│   ├── atomic/                  # 原子操作
│   └── compiler/                # 编译器差异、属性宏、弱符号等
│
├── board/                       # 板卡级适配
│   ├── board_a/
│   │   ├── board_config.h
│   │   ├── pin_map.h
│   │   └── peripheral_config.c
│   └── board_b/
│       ├── board_config.h
│       ├── pin_map.h
│       └── peripheral_config.c
│
├── config/                      # 产品级配置和功能开关
│   ├── app_config.h
│   ├── feature_config.h
│   ├── protocol_config.h
│   ├── parameter_config.h
│   └── fault_config.h
│
├── third_party/                 # 第三方库，不直接修改源码
│   ├── cmsis/
│   ├── freertos/
│   ├── littlefs/
│   ├── nanopb/
│   └── mbedtls/
│
├── generated/                   # 工具生成代码，如 CubeMX、代码生成器输出
│
├── bootloader/                  # Bootloader / IAP / OTA 相关工程或模块
│   ├── boot_app/
│   ├── upgrade_protocol/
│   └── image_verify/
│
├── test/                        # 单元测试、协议测试、仿真测试
│   ├── unit/
│   ├── integration/
│   ├── mock/
│   └── fixtures/
│
├── tools/                       # 构建、烧录、打包、日志解析等辅助工具
│   ├── build.sh
│   ├── flash.sh
│   ├── pack_firmware.py
│   ├── version_gen.py
│   └── log_parser.py
│
├── docs/                        # 工程文档
│   ├── architecture.md
│   ├── protocol.md
│   ├── fault_code.md
│   ├── parameter_table.md
│   ├── release_note.md
│   └── bringup.md
│
├── CMakeLists.txt               # 推荐使用 CMake 管理构建
├── README.md
└── .gitignore
```

---

## 4. 分层架构说明

### 4.1 app：应用层

应用层负责产品主流程、任务编排和状态调度。

适合放在 `app/` 的内容：

- 系统初始化流程
- RTOS 任务入口
- 主状态机调度
- 模块启动顺序
- 周期任务编排

应用层不应直接访问 HAL、寄存器或具体外设句柄。

### 4.2 service：服务层

服务层负责产品级业务能力封装。每个服务模块应对外提供清晰 API。

示例：

```c
void Motor_Init(void);
int Motor_SetTargetSpeed(int16_t rpm);
int Motor_Stop(void);
MotorState_t Motor_GetState(void);
```

服务层可以依赖中间件、驱动层和平台层，但不应依赖 `app/`。

### 4.3 middleware：中间件层

中间件应尽量做到平台无关、硬件无关、可复用、可测试。

示例：

```c
uint16_t Crc16_Calc(const uint8_t *data, uint16_t len);
int RingBuffer_Write(RingBuffer_t *rb, const uint8_t *data, uint16_t len);
float Pid_Update(Pid_t *pid, float target, float feedback);
```

### 4.4 driver：驱动层

驱动层负责屏蔽芯片、外设和外部器件差异。

建议拆分为：

- `bsp/`：板载 MCU 外设抽象。
- `device/`：外部器件驱动。
- `hal_port/`：厂商 SDK 适配。

### 4.5 platform：平台层

平台层解决不同 RTOS、编译器、芯片平台之间的差异。

典型接口：

```c
uint32_t Platform_GetTickMs(void);
void Platform_DelayMs(uint32_t ms);
void Platform_EnterCritical(void);
void Platform_ExitCritical(void);
```

---

## 5. 核心模块设计

### 5.1 BSP 硬件抽象

BSP 是业务代码和硬件之间的边界。

推荐接口风格：

```c
int Bsp_UartSend(uint8_t port, const uint8_t *data, uint16_t len);
int Bsp_CanSend(uint8_t channel, uint32_t id, const uint8_t *data, uint8_t len);
int Bsp_GpioWrite(uint16_t pin, uint8_t level);
uint32_t Bsp_GetTickMs(void);
```

要求：

- BSP 内部可以使用 HAL、LL 或寄存器。
- BSP 外部不暴露厂商句柄，如 `UART_HandleTypeDef`。
- BSP 接口返回统一错误码。

### 5.2 通信协议

通信协议应拆分为编解码、传输和业务处理。

```text
protocol parser  ->  command dispatcher  ->  service handler
```

建议：

- 协议解析器不直接操作硬件。
- 通信任务负责收发数据。
- 命令处理函数调用服务层接口。
- 协议文档放在 `docs/protocol.md`。

### 5.3 参数管理

参数管理用于保存设备配置、校准值、运行策略等。

建议能力：

- 参数默认值
- 参数范围校验
- 参数版本号
- 掉电保存
- CRC 校验
- 恢复出厂设置
- 参数升级兼容

### 5.4 日志系统

不要在业务代码中直接使用 `printf`。统一使用日志宏：

```c
LOG_DEBUG("sensor raw=%d", raw);
LOG_INFO("motor start");
LOG_WARN("voltage low: %d mV", voltage);
LOG_ERROR("flash write failed: %d", ret);
```

日志系统应支持：

- 日志等级
- 模块标签
- 时间戳
- 多输出后端，如 UART、RTT、Flash、CAN
- 编译期裁剪

### 5.5 故障管理

故障管理模块负责故障检测、故障记录、故障恢复和故障上报。

建议包含：

- 故障码定义
- 故障等级
- 触发条件
- 恢复条件
- 故障锁存
- 故障快照
- 故障上报接口

故障码应文档化到 `docs/fault_code.md`。

### 5.6 状态机

复杂业务应使用显式状态机，避免大量散乱的标志位。

示例状态：

```text
INIT -> IDLE -> RUNNING -> FAULT -> RECOVERY -> IDLE
```

要求：

- 状态定义集中。
- 状态切换条件明确。
- 异常状态有兜底处理。
- 关键状态切换记录日志。

### 5.7 看门狗与任务健康监控

工业级工程不应简单在主循环中固定喂狗。

建议：

- 每个关键任务上报心跳。
- 看门狗服务统一判断任务健康状态。
- 只有所有关键任务健康时才喂硬件看门狗。
- 复位前尽量记录故障原因。

### 5.8 Bootloader / OTA

需要远程升级或现场升级的产品，应规划 Bootloader 和固件镜像格式。

建议包含：

- 固件头信息
- 固件长度
- 固件版本
- 目标硬件版本
- CRC / hash
- 签名校验，视安全需求选择
- A/B 分区或回滚机制，视 Flash 资源选择

### 5.9 版本信息

固件应内置版本信息：

```c
#define FW_VERSION_MAJOR       1
#define FW_VERSION_MINOR       0
#define FW_VERSION_PATCH       0
#define FW_BUILD_NUMBER        1
#define FW_GIT_COMMIT_HASH     "abcdef0"
#define FW_BUILD_TIME          "2026-06-10 07:32:00"
```

版本信息应能通过通信协议读取，方便现场排查。

### 5.10 数据存储

数据存储模块负责参数、日志、故障记录、校准数据等持久化。

要求：

- 上层不直接操作 Flash / EEPROM。
- 写入前做边界和合法性检查。
- 关键数据带版本和校验。
- 频繁写入场景考虑磨损均衡。

---

## 6. 代码管理规范

### 6.1 模块文件组织

一个模块推荐结构：

```text
module_name/
├── module_name.c
├── module_name.h
├── module_name_config.h
├── module_name_port.c
└── module_name_port.h
```

说明：

- `module_name.h`：对外接口。
- `module_name.c`：核心实现。
- `module_name_config.h`：配置项。
- `module_name_port.*`：平台或硬件适配。

小模块可以只保留 `.c` 和 `.h`。

### 6.2 头文件规范

头文件只暴露外部必须知道的内容：

- 公共类型
- 公共宏
- 公共 API

不应在头文件中暴露内部状态变量。

推荐：

```c
typedef enum {
    MOTOR_STATE_IDLE = 0,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_FAULT,
} MotorState_t;

void Motor_Init(void);
int Motor_SetTargetSpeed(int16_t rpm);
MotorState_t Motor_GetState(void);
```

不推荐：

```c
extern int16_t g_motor_pwm;
extern uint8_t g_motor_fault_flag;
```

### 6.3 全局变量约束

优先使用模块私有上下文：

```c
typedef struct {
    int16_t target_speed;
    int16_t current_speed;
    MotorState_t state;
} MotorContext_t;

static MotorContext_t s_motor;
```

确实需要跨模块访问的数据，应通过函数接口读取或修改。

### 6.4 统一错误码

建议全工程使用统一错误码：

```c
typedef enum {
    ERR_OK = 0,
    ERR_NULL_PTR = -1,
    ERR_INVALID_PARAM = -2,
    ERR_TIMEOUT = -3,
    ERR_BUSY = -4,
    ERR_NO_MEMORY = -5,
    ERR_NOT_READY = -6,
    ERR_HW_FAILED = -7,
    ERR_CHECK_FAILED = -8,
} ErrorCode_t;
```

### 6.5 命名规范

建议采用统一前缀：

```text
模块名_动作
```

示例：

```c
Motor_Init();
Motor_Start();
Motor_Stop();
Sensor_Read();
Parameter_Save();
Fault_Report();
```

文件命名建议使用小写加下划线：

```text
motor_control.c
protocol_parser.c
fault_manager.c
```

### 6.6 配置宏规范

配置宏应集中定义，命名清晰：

```c
#define APP_ENABLE_CAN                 1
#define APP_ENABLE_UART_DEBUG          1
#define MOTOR_MAX_SPEED_RPM            3000
#define SENSOR_SAMPLE_PERIOD_MS        10
#define FAULT_RECORD_MAX_COUNT         32
```

不要在业务逻辑中直接出现不明含义的数字。

### 6.7 第三方库管理

第三方库放在 `third_party/`。

要求：

- 尽量不直接修改第三方源码。
- 如必须修改，应保留补丁说明。
- 记录来源、版本、许可证。
- 对外封装一层适配接口，避免业务代码直接绑定第三方库。

### 6.8 自动生成代码管理

由工具生成的代码放在 `generated/`。

要求：

- 手写代码不要直接混入生成目录。
- 如果工具允许用户代码区，应明确标记。
- 生成代码更新后，需要检查外设初始化、时钟配置和中断配置差异。

---

## 7. Git 分支与提交规范

### 7.1 分支模型

推荐分支：

```text
main              # 稳定发布分支
develop           # 日常集成分支
feature/xxx       # 新功能开发
bugfix/xxx        # 普通问题修复
release/x.y.z     # 发布准备
hotfix/xxx        # 线上紧急修复
```

小团队可以简化为：

```text
main
feature/xxx
bugfix/xxx
```

### 7.2 提交信息

推荐格式：

```text
type(scope): subject
```

常见类型：

```text
feat: 新功能
fix: 修复问题
refactor: 重构
test: 测试
docs: 文档
chore: 构建、脚本、杂项
style: 格式调整
```

示例：

```text
feat(motor): add speed closed-loop control
fix(protocol): handle crc error frame
docs(fault): update fault code table
chore(build): add firmware package script
```

### 7.3 标签与版本

发布版本应打 Git tag：

```text
v1.0.0
v1.1.0
v1.1.1
```

建议版本语义：

- 主版本：不兼容变更或重大架构调整。
- 次版本：新增功能。
- 修订版本：问题修复。

---

## 8. 构建、测试与发布

### 8.1 构建

推荐使用 CMake 或 Make 管理构建。

构建脚本建议放在 `tools/`：

```bash
./tools/build.sh
```

构建输出建议放在：

```text
build/
dist/
```

### 8.2 测试

推荐测试范围：

- 协议编解码测试
- CRC 测试
- 参数校验测试
- 状态机测试
- 故障逻辑测试
- 算法模块测试

测试命令示例：

```bash
./tools/test.sh
```

### 8.3 静态检查

建议引入：

- 编译器警告：`-Wall -Wextra`
- clang-tidy
- cppcheck
- MISRA C 检查，视项目要求选择

### 8.4 固件打包

发布产物建议包含：

```text
firmware_v1.0.0.bin
firmware_v1.0.0.hex
firmware_v1.0.0.elf
firmware_v1.0.0.map
firmware_v1.0.0.json
release_note_v1.0.0.md
```

其中 JSON 元信息可包含：

```json
{
  "name": "product_firmware",
  "version": "1.0.0",
  "git_commit": "abcdef0",
  "build_time": "2026-06-10 07:32:00",
  "target_board": "board_a",
  "hardware_version": "rev_a",
  "crc32": "0x12345678"
}
```

### 8.5 发布检查

发布前至少确认：

- 编译无错误。
- 关键警告已处理。
- 单元测试通过。
- 固件版本号正确。
- 协议版本号正确。
- 参数表版本号正确。
- Bootloader 兼容性确认。
- release note 已更新。
- 固件包可烧录、可启动、可回读版本。

---

## 9. 新工程创建流程

创建新下位机工程时，建议按以下步骤执行：

1. **确定产品边界**
   - 使用什么 MCU / SoC。
   - 是否使用 RTOS。
   - 通信接口有哪些。
   - 是否需要 Bootloader / OTA。
   - 是否需要参数掉电保存。

2. **创建基础目录**
   - 按本文档第 3 节创建目录。
   - 先保留空目录说明文件，如 `.gitkeep`。

3. **建立平台层**
   - 实现 tick、delay、critical、os adapter。
   - 明确裸机或 RTOS 的差异。

4. **建立 BSP 层**
   - 封装 UART、CAN、GPIO、ADC、PWM、Timer。
   - 禁止业务层直接使用 HAL 句柄。

5. **建立日志和错误码**
   - 先统一错误码。
   - 再统一日志输出。
   - 后续模块全部使用统一接口。

6. **建立配置体系**
   - 创建 `app_config.h`、`feature_config.h`、`board_config.h`。
   - 明确产品差异通过配置表达。

7. **建立核心服务**
   - 通信服务。
   - 参数服务。
   - 故障服务。
   - 看门狗服务。
   - 业务服务。

8. **补充测试**
   - 优先测试无硬件依赖模块。
   - 协议、参数、CRC、状态机必须优先覆盖。

9. **建立发布流程**
   - 自动生成版本号。
   - 打包固件。
   - 输出 release note。
   - 固件产物归档。

10. **持续维护文档**
    - 架构说明。
    - 通信协议。
    - 参数表。
    - 故障码。
    - 调试和烧录说明。

---

## 10. 推荐实践清单

新项目落地时，可以用下面的清单做自检。

### 架构

- [ ] 业务代码不直接依赖 HAL、LL、寄存器或厂商 SDK。
- [ ] `app`、`service`、`middleware`、`driver`、`platform` 分层明确。
- [ ] 下层模块不反向调用上层模块。
- [ ] 板级差异集中在 `board/` 和 `config/`。

### 模块

- [ ] 每个模块有清晰的 `.h` 对外接口。
- [ ] 模块内部状态使用 `static` 限制作用域。
- [ ] 跨模块访问通过 API，而不是直接访问全局变量。
- [ ] 复杂业务使用状态机表达。

### 质量

- [ ] 使用统一错误码。
- [ ] 使用统一日志接口。
- [ ] 关键故障有故障码和恢复策略。
- [ ] 看门狗由任务健康状态统一驱动。
- [ ] 核心中间件有单元测试。

### 发布

- [ ] 固件内置版本号、构建时间和 Git commit。
- [ ] 发布产物包含 bin、hex、elf、map 和元信息。
- [ ] release note 记录功能、修复、兼容性和已知问题。
- [ ] 发布版本打 Git tag。

### 文档

- [ ] `docs/architecture.md` 描述整体架构。
- [ ] `docs/protocol.md` 描述通信协议。
- [ ] `docs/fault_code.md` 描述故障码。
- [ ] `docs/parameter_table.md` 描述参数表。
- [ ] `docs/bringup.md` 描述调试、烧录和启动流程。

---

## 11. 最小可落地版本

如果项目初期不想一次性建立完整工业级框架，可以先创建最小版本：

```text
project/
├── app/
├── service/
├── middleware/
├── driver/
├── platform/
├── board/
├── config/
├── third_party/
├── test/
├── tools/
└── docs/
```

随后按项目需要逐步补充：

- `bootloader/`
- `generated/`
- `service/upgrade/`
- `service/fault/`
- `service/parameter/`
- CI 和自动化发布流程

原则是：**可以从轻量开始，但边界不要混乱；可以暂时不完整，但方向要正确。**
