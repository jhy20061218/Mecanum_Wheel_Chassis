# JHY 麦克纳姆轮底盘预测与模拟课程设计

本作业在所给底盘代码基础上，增加电脑端 C11 仿真实验、参数与事件输入、组件状态监测、CSV 记录、结果分析、报告生成和中文文本编辑。电脑端直接复用原工程逆运动学源码；固件增加默认停机的实验监督、故障联锁和串口遥测。

**本次未连接或烧录实物。`Simulation/results` 中所有数据均为实际运行 C 程序得到的软件仿真结果。**

## 作业成果入口

- `docs/设计文档.docx` 与 `docs/实验工作报告.docx`：提交用文档，另有可编辑 Markdown 源文件。
- `Simulation/README.md`：电脑程序运行说明、完整参数表、文本编辑命令及局限性。
- `Simulation/results/`：六种场景的原始 CSV、JSON 指标、自动报告及输入快照。
- `docs/验证记录.json`：独立逐行数据校验与端到端回归记录。
- `JHY/固件实验说明.md`：实物接线、参数命令、遥测协议、安全流程与构建证据。

## 快速运行电脑端 C 程序

Windows 可直接使用随包提供的可执行文件。请先进入 `Simulation` 目录，再运行：

```powershell
.\build\mecanum_sim.exe --self-test
.\build\mecanum_sim.exe --config config/default.cfg --output results/demo
.\build\mecanum_sim.exe --config config/default.cfg --output results/realtime --realtime
```

默认快速计算，加 `--realtime` 可按近似实时节拍观察四轮状态。自行编译时需配置 GCC（MinGW），在 `Simulation` 目录执行 `powershell -ExecutionPolicy Bypass -File .\build.ps1 -RunAll`。C 程序本身不需要 Python 或第三方 C 库；Python 仅用于可选回归和文档生成。

报告文本编辑支持中文文件名及内容，建议先复制自动报告再编辑：

```powershell
Copy-Item results/default/report.md results/default/report_edited.md
.\build\mecanum_sim.exe --edit-report results/default/report_edited.md append "补充实验分析"
.\build\mecanum_sim.exe --edit-report results/default/report_edited.md replace "补充实验分析" "修改后的实验分析"
.\build\mecanum_sim.exe --edit-report results/default/report_edited.md delete "修改后的实验分析"
```

## 实物采集与固件构建

在工程根目录运行 `tools/Capture-Telemetry.ps1 -Port COM3 -DurationSec 30 -Output hardware_test.csv` 可只读采集串口，COM3 应换为实际端口。脚本不会发送使能或运动命令。采集参数为 USART1 420000 baud、8N1，每 20 ms 一帧、26 列 CSV，每 5 s 重复表头。固件默认未使能，具体操作先阅读 `JHY/固件实验说明.md`。

配置 ARM GCC、CMake 与 Ninja 后，运行 `JHY/Build-Firmware.ps1` 只构建不烧录；脚本处理本机 CMake 在中文路径中的异常。软件急停不能代替独立硬件急停，电脑端 PID 不能直接写入实车速度环。

以下保留原硬件工程的架构与基础参数说明。新增课程设计应用署名为 JHY，第三方版权保留。

## 原硬件工程说明

基于 STM32F407 的四轮麦克纳姆轮底盘运动控制系统，使用四个 DJI M3508 直流无刷电机（搭配 C620 电调），通过 CAN 总线实现 1 kHz 的实时闭环控制。

## 硬件平台

| 组件 | 型号/参数 |
|------|-----------|
| 主控 | STM32F407IGH6 (Cortex-M4F, 168 MHz) |
| 电机 | DJI RoboMaster M3508 × 4 |
| 电调 | DJI C620 × 4 |
| 通信 | CAN 总线, 1 Mbps |
| 调试器 | CMSIS-DAP (DAPLink) / OpenOCD |

## 软件架构

```
.
├── Core/                   # STM32CubeMX HAL 层代码
│   ├── Inc/                # 外设头文件 (can, tim, usart, gpio, dma)
│   └── Src/                # 外设源文件 + main.c
├── Drivers/                # CMSIS + STM32F4xx HAL 驱动库
├── JHY/                    # 用户应用层代码
│   ├── Inc/
│   │   ├── alg_PID.h              # PID 控制器
│   │   ├── drv_math.h             # 数学工具库
│   │   ├── drc_can.h              # CAN 总线驱动
│   │   ├── dvc_DJI_M3508.h        # DJI M3508 电机驱动
│   │   └── Mecanum_Wheel_Chassis.h # 麦克纳姆轮运动学
│   └── Src/
│       ├── alg_PID.c
│       ├── drv_math.c
│       ├── drc_can.c
│       ├── dvc_DJI_M3508.c
│       └── Mecanum_Wheel_Chassis.c
├── cmake/                  # CMake 工具链文件
├── Mecanum_Wheel_Chassis.ioc   # CubeMX 工程配置
├── STM32F407XX_FLASH.ld    # 链接脚本
└── CMakeLists.txt           # 顶层 CMake 构建文件
```

## 模块说明

### PID 控制器 (`alg_PID`)

功能完备的 PID 控制器，支持以下特性：

- **P / I / D / F**（前馈）四项独立调节
- **死区**（Dead Zone）：误差低于阈值时忽略
- **变速积分**：积分增益在误差区间 [A, A+B] 内线性缩放
- **积分分离**：误差超过阈值时清零积分项
- **微分先行**（Derivative-on-Measurement）：减少设定值突变引起的微分冲击
- **输出限幅** + **积分限幅**

### CAN 驱动 (`drc_can`)

CAN 总线抽象层，管理 CAN1/CAN2 的收发：

- 预分配 TX 缓冲区，支持 M3508 (0x200)、M2006 (0x1FF)、GM6020 (0x1FE) 等电机
- 硬件掩码滤波器：接收 ID 范围为 `0x200 ~ 0x20F`
- TIM8 驱动的 1 kHz 定时发送
- HAL CAN 接收回调分发

### DJI M3508 电机驱动 (`dvc_DJI_M3508`)

完整的 M3508/C620 电机控制驱动：

- **四种控制模式**：开环 / 力矩 / **速度** / 角度（级联 PID）
- CAN 反馈解析：编码器、转速、力矩电流、温度
- **多圈编码器**：自动处理 0–8191 翻转
- **心跳检测**：超时自动停机并复位 PID 积分
- 支持减速比配置（默认 3591/187 ≈ 19.2:1）

### 麦克纳姆轮运动学 (`Mecanum_Wheel_Chassis`)

四轮麦克纳姆轮逆运动学求解：

- 输入：目标速度 `(vx, vy, ω)`（横向、纵向、偏航角速度）
- 输出：各轮目标角速度 `(ω_LF, ω_LB, ω_RB, ω_RF)`
- **防饱和缩放**：任一车轮超速时，等比例缩放全部四轮以保持运动方向

```
            ↑ y (前)
            │
    LF ─────────── RF
    │              │
    │      ●       │  ← 底盘中心
    │              │
    LB ─────────── RB
            │
```

### 数学工具 (`drv_math`)

- 数值约束宏 (`ABS`, `LIMIT` 等)
- 大小端转换
- 校验和计算
- sinc 函数

## 控制回路

```
┌──────────┐    ┌─────────────┐    ┌──────────┐    ┌──────────┐
│ 速度指令  │───▶│ 逆运动学求解 │───▶│ PID 控制  │───▶│ CAN 发送 │
│(vx,vy,ω) │    │(Mecanum)    │    │(omega)   │    │(1 kHz)   │
└──────────┘    └─────────────┘    └──────────┘    └──────────┘
                                                         │
                                                    CAN Bus
                                                         │
┌──────────┐    ┌─────────────┐    ┌──────────┐         │
│ 状态估计  │◀───│ 编码器解算   │◀───│ CAN 接收 │◀────────┘
│(omega)   │    │(多圈追踪)   │    │(FIFO0)  │
└──────────┘    └─────────────┘    └──────────┘
```

- **控制频率**：1 kHz（TIM8 中断驱动）
- **控制方法**：速度环 (Omega) PID
- **反馈源**：M3508 磁编码器 (8192 counts/rev)

## 关键参数

| 参数 | 数值 | 说明 |
|------|------|------|
| 系统时钟 | 168 MHz | HSI + PLL |
| CAN 波特率 | 1 Mbps | CAN1 |
| 控制频率 | 1 kHz | TIM8 定时器 |
| 底盘半宽 (a) | 0.215 m | 横向 |
| 底盘半长 (b) | 0.2215 m | 纵向 |
| 车轮半径 (R) | 0.077 m | 麦克纳姆轮 |
| 最大轮速 | 15.0 rad/s | 防饱和限幅 |
| 减速比 | 3591/187 | M3508 官方减速箱 |
| 编码器分辨率 | 8192 | 每圈脉冲数 |

## 构建与烧录

### 前置依赖

- **ARM 工具链**：`arm-none-eabi-gcc` (GCC ARM Embedded)
- **构建工具**：CMake 3.22+, Ninja
- **调试器**：OpenOCD (CMSIS-DAP)

### 构建

```bash
# Debug 构建
cmake --preset Debug
cmake --build build/Debug

# Release 构建
cmake --preset Release
cmake --build build/Release
```

### 烧录

```bash
openocd -f MicroLink_F4.cfg -c "program build/Debug/Mecanum_Wheel_Chassis.elf verify reset exit"
```

### IDE

项目支持 **CLion** 直接打开，已包含 `.idea/` 配置和 CMakePresets。

也可以通过 **STM32CubeMX** 打开 `Mecanum_Wheel_Chassis.ioc` 修改硬件配置。

---

## License

This project is for educational and research purposes.
