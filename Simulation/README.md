# JHY 麦克纳姆轮底盘预测与仿真实验

本目录是基于原 STM32 麦轮底盘工程扩展的 **纯 C11 上位机仿真实验程序**。它直接编译链接 `../JHY/Src/alg_mecanum_wheel_chassis.c`，复用原四轮逆运动学与等比例限幅，再增加闭环电机模型、参数与事件、状态监测、CSV 数据采集、可靠性分析和报告编辑。

**本程序不连接 CAN、不驱动电机。所有 `results` 数据均为软件仿真。** 实物实验步骤及固件接口见工程 `docs` 文档；报告不能作为已经进行实车实验的证明。

## 1. 快速运行

以下命令均在本 `Simulation` 目录执行。Windows 使用已安装的 GCC（MinGW）；核心程序不需要 Python、MATLAB 或第三方 C 库。

```powershell
# 编译、核心自检并运行全部六组实验
powershell -ExecutionPolicy Bypass -File .\build.ps1 -RunAll

# 单独运行默认实验，按模拟时间每秒输出状态
.\build\mecanum_sim.exe --config config/default.cfg --output results/default

# 演示近似实时节拍（12秒实验约12秒墙钟时间）
.\build\mecanum_sim.exe --config config/default.cfg --output results/realtime --realtime

# 显示命令帮助
.\build\mecanum_sim.exe --help
```

`--realtime` 在每步计算后等待一个步长，计算开销和系统调度会使总墙钟时间略长，不是硬实时调度。默认不等待，便于快速批量预测。

也可独立使用 CMake。此子项目只构建电脑程序，与 STM32 的 ARM 工具链工程分开配置：

```powershell
cmake -S . -B build-cmake -G Ninja
cmake --build build-cmake
ctest --test-dir build-cmake --output-on-failure
```

本机附带的 JetBrains CMake 在当前含中文路径中配置时退出异常；同一份源码在独立 ASCII 路径已通过 CMake 构建与 CTest。当前目录可直接使用已验证的 `build.ps1`；若选用该版本 CMake，可将完整工程复制到纯英文路径后运行上述命令。

Linux/macOS 使用相同 CMake 命令，运行文件通常为 `build-cmake/mecanum_sim`。Windows 版本另调用系统 `shell32` API 解析 UTF-16 参数，并将 UTF-8 文件路径转换为宽字符路径，支持中文报告内容与中文目录。

## 2. 作业要求对应功能

| 要求 | 本项目实现 |
|---|---|
| 参数设定与调整 | `key=value` 配置；`--write-config` 导出完整默认配置；事件 CSV 在运行期间改变速度/PID/滑移 |
| 实验状态与数据采集 | 每步 50 列 CSV；控制台四轮轮速、温度、位姿和故障状态；保存初始配置与事件 |
| 预测与模拟 | 共享原逆运动学，PID 速度闭环、一阶电机、滑移、噪声、正运动学与世界位姿积分 |
| 安全实验 | 输入参数范围校验；过温、反馈超时、急停、超速、非有限数值检测；故障锁存后四路驱动输出为零 |
| 准确性与可靠性分析 | 位置和累计航向 RMSE、最大误差、关闭噪声的步长减半终点差、自检、重复性及故障回归 |
| 工作报告与文本编辑 | 自动生成中文 Markdown 报告；UTF-8 文本追加/删除/替换；根目录文档提供设计文档与实验报告 |

## 3. 目录与输入

```text
Simulation/
  src/                 C11 模块
    main.c             CLI、运行编排与步长减半比较
    config.c           参数和事件读取、格式与范围校验
    sim.c              模型、闭环、监测、CSV与报告生成
    editor.c           UTF-8报告文本处理
    io.c               Windows/POSIX路径处理
    tests.c            内置模型与安全自检
  config/              正常/滑移/掉线/过温/急停/限幅实验方案
  tests/regression.py  可选Python标准库端到端回归
  build.ps1            Windows一键构建与实验
  results/             已执行实验输出
```

### 配置文件

配置使用 UTF-8（允许 BOM），每行 `参数名=数值`，`#` 后为注释。未写出的参数沿用默认值；未知名称、NaN、Inf、异常范围会报错，不开始实验。事件文件路径按**启动程序时的工作目录**解析，因此建议始终从 `Simulation` 目录运行。

```powershell
.\build\mecanum_sim.exe --write-config config/my_experiment.cfg
# 用文本编辑器修改 my_experiment.cfg
.\build\mecanum_sim.exe --config config/my_experiment.cfg --output results/my_experiment
```

| 参数 | 默认值 | 单位及有效范围 |
|---|---:|---|
| `duration` | 12 | s；0 < duration ≤ 600，且总步数 ≤ 1,000,000 |
| `dt` | 0.01 | s；0.0001 ≤ dt ≤ 0.05，同时 dt ≤ motor_tau/5 |
| `a` / `b` | 0.215 / 0.2215 | m；半宽/半长，0.001 < 值 ≤ 5 |
| `radius` | 0.077 | m；0.001 < 值 ≤ 1 |
| `max_wheel_speed` | 15 | rad/s；0 < 值 ≤ 100，逆运动学等比例限幅 |
| `vx` / `vy` / `omega` | 0.25 / 0.35 / 0.2 | m/s、m/s、rad/s；各自绝对值 ≤ 10 |
| `kp` / `ki` / `kd` | 0.08 / 0.6 / 0.0004 | 归一化驱动控制增益；分别 [0,10]、[0,100]、[0,1] |
| `motor_tau` / `motor_gain` | 0.12 / 20 | s / rad/s；5dt ≤ tau ≤ 10，0 < gain ≤ 200 |
| `load` / `slip` | 0.08 / 0 | 无量纲；[0,0.9) / [0,1) |
| `noise` | 0.02 | 编码器均匀噪声半宽，rad/s；[0,2] |
| `ambient` | 25 | °C；[-20,50] |
| `initial_temperature` | 25 | °C；ambient ≤ 值 ≤ 150 |
| `max_temperature` | 65 | °C；ambient < 值 ≤ 120，课程模型阈值 |
| `feedback_timeout` | 0.1 | s；dt ≤ 值 ≤ 10 |
| `max_current` | 8 | A；0 < 值 ≤ 20，控制量对应的估计电流上限 |
| `print_interval` | 1 | s；值 ≥ dt |
| `emergency_time` | -1 | -1 表示禁用；否则 0 ≤ 值 < duration |
| `seed` | 2026 | 1 至 4294967295 的整数 |
| `events` | 空字符串 | 事件文件路径；`config/default.cfg` 设置为 `config/motion.csv` |

输入范围保证有限计算与教学模型的基本使用条件，不保证任意 PID 参数都能稳定跟踪。总步数是按初始 `dt` 校验；可靠性比较内部会再运行同一方案的无噪声版本及 `dt/2` 版本。

### 运行期间调整与故障注入

事件文件每行 `time_s,key,value`，必须按时间非递减排列，时间范围 `0 ≤ time_s < duration`。同一时刻可连续写多项。事件在首个到达其时刻的控制步开始执行，量化延迟小于一个控制步长。

```csv
1.5,vx,0.4
1.5,vy,0
2,kp,0.1
3,slip,0.25
4,disconnect,1
5,estop,1
```

支持 `vx`、`vy`、`omega`、`kp`、`ki`、`kd`、`slip`，范围与配置表相同；`disconnect=1/2/3/4` 分别使 LF/LB/RB/RF 反馈掉线，0 恢复全部反馈；`temperature_LF/LB/RB/RF` 注入该轮温度，范围 [-20,150] °C；`estop` 只接受 1。故障一旦锁存，本次运行不会因恢复反馈而自动恢复驱动，需要重新开始实验。

默认 `motion.csv` 在 3、6、9 s 改变运动指令，演示参数动态调整；其他配置单独测试 25% 滑移、2 s 掉线、2 s 过温、2 s 急停及全程限幅。

## 4. 数学模型与数据语义

轮序 LF、LB、RB、RF，与原工程一致。原工程约定 x 向右、y 向前、偏航按原运动学代数符号。实物必须架空验证轮号、旋向与减速比。

逆运动学直接调用共享源码。令 k=a+b，其未限幅轮速为：

```text
wLF = (vx - vy - k*omega)/R
wLB = (vx + vy - k*omega)/R
wRB = (vx - vy + k*omega)/R
wRF = (vx + vy + k*omega)/R
```

任一轮超限时，四轮同时乘 `max_wheel_speed / max(abs(wi))`。正运动学为：

```text
vx    = R*( wLF + wLB + wRB + wRF)/4
vy    = R*(-wLF + wLB - wRB + wRF)/4
omega = R*(-wLF - wLB + wRB + wRF)/(4*k)
```

每轮电机采用 `dw/dt = [motor_gain*(1-load)*u-w]/motor_tau` 的显式欧拉模型。控制量 `u` 是 [-1,1] 的归一化驱动指令，并不直接代表 C620 电流发送值。PID 使用稳态速度前馈、测量微分和条件积分防饱和：

```text
e = target - measured
Ic = clip(I + e*dt, -1, 1)
raw = target/motor_gain + kp*e + ki*Ic - kd*(measured-previous)/dt
u = clip(raw, -1, 1)
```

仅在输出未饱和或积分有助于退出饱和时接受 `Ic`。地面有效轮速为 `(1-slip)*w`，测量为 `w+uniform(-noise,+noise)`，再由正运动学得到速度。位姿使用步中航向旋转至世界坐标并积分。温度采用 `dT/dt = 0.025*I_est² - 0.08*(T-ambient)`、`I_est=abs(u)*max_current`。

保留三条轨迹：

- `reference`：限幅后的持续任务参考；故障发生后仍继续前进，表示原计划目标。
- `actual`：电机动态和滑移下的模型真值。
- `odometry`：有噪声且未补偿滑移的编码器里程计。

故障后的任务参考仍在运动，误差会扩大；这用于评价任务偏离，不能据此判断“停机保护没有生效”。停机保护须检查四路 `control=0` 和四路施加 `target=0`，随后实际轮速按惯性模型衰减。

## 5. 输出与评估

每次输出目录包含：

| 文件 | 用途 |
|---|---|
| `telemetry.csv` | 每个控制步一行，共 50 列；可用 Excel、Python 或其他绘图工具查看 |
| `summary.json` | 结构化结果、RMSE、故障时刻、终点位姿、步长敏感性 |
| `report.md` | 自动生成、可编辑的中文实验报告 |
| `used_config.cfg` | 本次完整初始配置（动态调整另见事件） |
| `used_events.csv` | 本次解析执行的事件列表 |

CSV 前 18 列：`time_s`、三轴任务指令、参考位姿、实际位姿、里程计位姿、位置误差、累计航向误差、滑移率、任务限幅标记、全局故障状态。后 32 列为四轮各 8 项：施加目标轮速、实际轮速、测量轮速、归一化控制量、估计电流、模拟温度、反馈年龄、在线标记。具体字段名以 CSV 首行英文表头为准，单位已在表头标明。

时间戳为控制步结束时刻；测量值在该步开始采集，实际速度/位姿/温度为步末模型状态，`fault_time_s` 为检测到故障的控制步开始时刻。`omega_command`、`vx_command`、`vy_command` 保留用户任务指令；四轮 `target_radps` 是考虑故障联锁后的施加目标。故障行的任务参考与施加目标因此不同。`yaw_error_rad` 是未折回到 [-π,π] 的**累计航向差**。

位置 RMSE 是时间加权的 `sqrt(sum((dx²+dy²)*h)/duration)`；累计航向 RMSE 类似。两者是相对理想任务参考的跟踪误差，不能当作与实车真值比较的预测精度。`dt_halving_*` 指标关闭噪声，比较同方案 dt 与 dt/2 的最终实际位姿，避免随机采样密度变化掩盖数值误差。单次差值是步长敏感性检查，不构成完整收敛阶证明或统计置信区间。

重放保存的输入时，从原 `Simulation` 目录使用 `used_config.cfg`；若移走输出目录，应同时修改 `events` 为 `used_events.csv` 的新路径。

## 6. 报告文本追加、删除、修改

```powershell
# 建议先复制报告作为编辑副本
Copy-Item results/default/report.md results/default/report_edited.md
.\build\mecanum_sim.exe --edit-report results/default/report_edited.md append "补充：已核对四轮状态与参数。"
.\build\mecanum_sim.exe --edit-report results/default/report_edited.md replace "补充：已核对四轮状态与参数。" "补充：已完成轨迹和温度数据复核。"
.\build\mecanum_sim.exe --edit-report results/default/report_edited.md delete "补充：已完成轨迹和温度数据复核。"

# 多行长文本可先另存为UTF-8文件，再用@文件名传入
.\build\mecanum_sim.exe --edit-report results/default/report_edited.md append @extra_note.txt
```

删除与替换会处理所有精确匹配；空匹配和无匹配会报错，保持原文件。UTF-8 校验失败会拒绝操作。编辑结果先写同目录临时文件，完整成功后替换原文件；程序不创建历史备份。修改文字不会重新计算 CSV 或 summary.json，编辑结论后须保持数据与文字一致。

## 7. 验证与复现

```powershell
.\build\mecanum_sim.exe --self-test
python tests/regression.py
```

内置 C 自检覆盖逆/正运动学一致性、限幅、正常跟踪、滑移、急停、反馈超时、过温以及非法参数。可选端到端回归只用 Python 标准库，覆盖六组 CSV 结构/有限性/时序/联锁、固定种子逐字节复现、非法配置和终点事件拒绝、中文目录及 UTF-8 编辑。回归产物保存在 `build/regression_*`，不计入实验提交数据。

本机以 GCC `-std=c11 -O2 -Wall -Wextra -Wpedantic -Werror` 编译并通过内置自检与 82 项端到端检查。已执行结果：

| 场景 | 行数 | 位置 RMSE / m | 故障状态 | 故障时刻 / s |
|---|---:|---:|---|---:|
| default | 1200 | 0.01040543 | OK | 无 |
| slip（25%） | 1200 | 0.38568109 | OK | 无 |
| timeout | 600 | 0.74190420 | FEEDBACK_TIMEOUT_LF | 2.10 |
| overheat | 600 | 0.77070669 | OVERHEAT_RF | 2.00 |
| estop | 600 | 0.77070669 | ESTOP | 2.00 |
| saturation | 400 | 0.04463926 | OK | 无 |

## 8. 局限性与实物实验前检查

模型未含四轮受力耦合、滚子碰撞、地面非均匀摩擦、载荷转移、坡度、电池动态、CAN 仲裁或控制抖动。温度与电流仅教学估计，需实测辨识时间常数、增益、滑移、热参数和真实电流映射后再用于工程预测。固定种子可复现不等于统计可靠性；进一步研究应多种子、多载荷扫描，并用独立定位设备校验。

实际操作应先架空标定轮号/旋向/减速比，检查通信、供电、机械固定与独立急停，再以低速限流方式试验。掉线、过温等故障应优先在软件仿真或无驱动力台架注入，不能通过真实过热、撞击等危险方式验证。软件联锁不能代替硬件急停或独立断电保护。
