# 智能送药小车 · STM32F103RCT6

小车在药房识别病房号（1~8），自动把药品送到对应病房、点亮指示灯，再原路返回药房。
控制上使用**位置速度串级 PID** 驱动两路带编码器的直流减速电机，**五路灰度传感器**巡红白线，
**OpenMV 模板匹配**识别病房号，并配有一套 **OLED + 按键调试菜单**。

> 题目基本要求：把药品送到近端 / 中端 / 远端病房并返回，运送与返回各自需在 20 秒内完成
> （中端、远端病房号在分岔口随机摆放，需要现场识别）。

---

## 一、完成的功能

### 1. 送药主流程（路线表状态机）

整套流程实现在 `User/app/app_task.c`，把每个房间的动作用**路线表**描述，需要看识别结果分叉的地方用分支路线：

```c
static const app_step_t steps_deliver_A[] = {
  STEP_GO(75),            // 直行 75cm
  STEP_TURN(left_90),     // 原地左转 90°
  STEP_GO(36),            // 直行 36cm
  STEP_RED_ON(),          // 点亮红灯（送达提示）
  STEP_END()
};
```

| 任务 | 流程 |
|---|---|
| **近端病房 1 / 2** | 直行 75cm → 左/右转 90° → 直行 36cm → 亮红灯 |
| **中端病房 3 / 4** | 直行 115cm 到路口 → 识别数字判断偏左/偏右 → 直行 50cm → 左/右转 90° → 直行 36cm → 亮红灯 |
| **远端病房 5~8** | 115cm（未命中则继续直行 88cm 到远端路口）→ 再识别 → 进入左/右支线 → 48cm → 转 90° → 36cm → 第三次识别 → 52cm → 转 90° → 36cm → 亮红灯 |
| **返回药房** | 药品被取走后自动掉头原路返回：近端回 70cm、中端回 160cm、远端经过两次转向回 238cm，到药房亮绿灯 |
| **异常处理** | 每个识别点都有 500ms 超时，超时按预设分支继续（近端不识别、中端超时继续去远端），不会卡死 |

### 2. 运动控制

**串级 PID 控制链路**，两个轮子各自一套：

```
期望位置(cm) → [位置环 P] → 期望转速(rpm) → [速度环 PID] → PWM 占空比 → 电机
                    ↑                              ↑
              编码器累计脉冲                  单位时间脉冲数
```

- **位置速度串级 PID**：位置环（`Kp=0.18`）输出目标转速并限幅 110rpm，速度环（`Kp=20 / Ki=2.9 / Kd=8`）输出 PWM；位置环 40ms、速度环 20ms 一次
- **按厘米给目标**：`Car_go(75)` 直接把厘米换算成编码器脉冲（`2464` 脉冲/输出轴圈、轮径 6.5cm），不用手调脉冲数
- **原地差速转向**：`spin_Turn(left_90 / right_90 / back_180)`，按轮距 17.5cm 计算 90° 对应的弧长，同样是位置闭环
- **五路灰度巡线**：分段补偿表（`0 / ±400 / ±500`），最外侧两路用于识别路口和十字，补偿直接叠加到左右轮速度环输出上
- 两个轮子各自独立的位置环 + 速度环，打滑时靠巡线环自动纠正

### 3. 感知与识别

- **OpenMV 数字识别（模板匹配）**，两种模式：
  - `Find_Task = 1`：药房第一次识别房号，只搜索画面中间区域，识别 1~8
  - `Find_Task = 2`：行进中识别，整幅搜索，按匹配框位置判断数字**偏左还是偏右**，把结果回传给 STM32
- **32 个数字模板**（`OpenMV/templates/`）：每个数字一张主模板 + 远端 3~8 每个数字 4 个角度模板
- **载药检测**：红外对管接 ADC，连续采样判断"已装药 / 已取走"，据此切换送药与返回阶段

### 4. 人机交互与调试

- **OLED 多级菜单**（I2C，128×64）：

  | 菜单 | 页面 | 内容 |
  |---|---|---|
  | **TASK** | `phase` / `motion` / `route` / `all` | 当前阶段、运动标志位、识别与目标房间、一屏总览 |
  | **TEST** | `motor` / `Gray` / `openmv` / `Load` | 编码器转速、五路灰度与载药、OpenMV 识别结果、ADC 电压 |

- **3 个按键**：`KEY1` 翻页（到底回绕）、`KEY2` 进入下一级、`KEY3` 回主界面；另有一个板载按键走外部中断
- **野火 PID 上位机**（USART1）：在线改 PID 参数、把实际值回传画波形，用来整定串级 PID

---

## 二、硬件

| 部件 | 型号 / 参数 |
|---|---|
| 主控 | STM32F103RCT6（LQFP64），HSE 8MHz × PLL9 = **72MHz** |
| 电机 | JGB37-250 直流减速电机 ×2，12V，减速比 **1:56**，实测最高约 180rpm |
| 电机驱动 | L298N ×2 |
| 编码器 | 霍尔编码器，**11 线 / 电机圈**，定时器编码器接口 4 倍频 |
| 车轮 | 直径 **6.5cm**（周长 ≈ 20.42cm），轮距 17.5cm |
| 循迹 | 五路灰度传感器（红白线，开关量输出） |
| 视觉 | OpenMV4（UART 通信，模板匹配） |
| 显示 | 0.96" OLED，SSD1306，硬件 I2C，地址 0x78 |
| 载药检测 | 红外对射管 → ADC 采样 |
| 交互 | 3 个按键 + 1 个板载按键 |

### 引脚分配

| 功能 | 引脚 | 外设 |
|---|---|---|
| 电机 A（左）PWM | PB8 = AIN1 / PB9 = AIN2 | TIM4_CH3 / CH4，24kHz |
| 电机 A 编码器 | PA6 / PA7 | TIM3 编码器模式（4 倍频） |
| 电机 B（右）PWM | PB10 = BIN1 / PB11 = BIN2 | TIM2_CH3 / CH4，**部分重映射 2** |
| 电机 B 编码器 | PC6 / PC7 | TIM8 编码器模式 |
| 灰度传感器 | L2=PC3，L1=PC4，M=PC5，R1=PC8，R2=PC9 | GPIO 输入（高 = 压到红线） |
| 载药红外 | PA1 | ADC1_IN1 |
| OLED | PB6 = SCL / PB7 = SDA | I2C1 |
| 按键 KEY1/2/3 | PC0 / PC1 / PC2 | GPIO 上拉输入 |
| 板载按键 | PA0 | EXTI0 下降沿 |
| 指示灯 | PA8（绿，低有效）/ PB15（红）/ PA11（蓝）/ PA12（黄） | GPIO 推挽 |
| OpenMV | PC10 = TX / PC11 = RX | USART3，115200，部分重映射 |
| 上位机 | PA9 = TX / PA10 = RX | USART1，115200 |
| 调试 | PA13 / PA14 | SWD |

---

## 三、通信协议

### OpenMV（USART3，115200）

STM32 → OpenMV（约每 21ms 一帧，4 字节）：`"*" + 任务模式 + 目标数字 + "&"`

OpenMV → STM32（固定 7 字节）：

| 0x2C | 0x12 | Num | LoR | find_flag | Find_Task | 0x5B |
|---|---|---|---|---|---|---|
| 帧头 | 帧头 | 识别到的数字 | 1=偏左，2=偏右 | 是否识别到 | 回显模式 | 帧尾 |

### 野火 PID 上位机（USART1，115200）

帧头 `0x59485A53`，依次为 通道(1B) / 包长(4B) / 命令(1B) / 参数(4B×N) / 校验和(1B)，
支持在线修改 PID 参数、目标值，以及把实际值回传画波形。详见 `User/bsp_debug/Fire_protocol.c`。

---

## 四、编译与部署

### STM32（Keil MDK）

| 项 | 值 |
|---|---|
| 器件包 | `Keil.STM32F1xx_DFP` **2.4.1** |
| 器件 | STM32F103RC |
| 预定义宏 | `USE_HAL_DRIVER,STM32F103xE` |
| 编译器 | **ARM Compiler 5（AC5）** |

> 工程用的是 **ARM Compiler 5**。MDK 5.37 以后 AC5 不再随安装包提供，需要单独安装
> *Arm Compiler 5.06 update 7*；或在 `Options for Target → Target → ARM Compiler` 里切到 AC6 后重新编译。
>
> CubeMX 工程文件是 `Car_RCT6.ioc`（CubeMX 6.8.1 / FW_F1 V1.8.5）。
> **注意：用 CubeMX 重新生成代码会覆盖 `USER CODE BEGIN/END` 之外的内容**，
> 而本工程在 `main.c` 的 `USER CODE BEGIN WHILE` 段里放主循环、在 `stm32f1xx_it.c` 里放中断回调，重新生成前记得备份。

下载：J-Link / ST-Link 均可，`Options for Target → Debug` 里选择自己的调试器。

### OpenMV

1. 把 `OpenMV/main.py` 拷到 OpenMV 存储**根目录**（上电自动运行 `main.py`）
2. 把 `OpenMV/templates/` 里的 **32 个 `.pgm` 也拷到根目录**（代码里用的是 `/1.pgm` 这类绝对路径，不能放子文件夹）
3. 串口 UART3 / 115200，与 STM32 共地

---

## 五、调参入口

| 想改什么 | 改哪里 |
|---|---|
| 某段路走多远 / 左转还是右转 / 什么时候亮灯 | `User/app/app_task.c` 里的路线表（`STEP_GO / STEP_TURN / STEP_LIGHT / STEP_FIND`） |
| 识别超时时间 | `User/Control/control.h` 的 `WaitTime_ms`（默认 500ms） |
| 巡线补偿力度 | `User/GraySensor/bsp_graySensor.c` 的 `Light_GoStraight_control()` 补偿表 |
| 速度上限 | `User/Control/control.h` 的 `TARGET_SPEED_MAX`（默认 110rpm） |
| PID 参数 | `User/pid/bsp_pid.c` 的 `PID_param_init()`，或用野火上位机在线调 |
| 轮径 / 轮距 / 减速比 | `User/Control/control.c`、`User/Control/control.h` |
| 菜单显示内容 | `User/menu/menu.c` 的 `OLED_Display()` |
| OpenMV 匹配阈值 / ROI / 模板 | `OpenMV/main.py`、`OpenMV/templates/`（详见 `OpenMV/README.md`） |

上车前建议先看 OLED 调试页：**TASK** 页看任务阶段和运动状态，**TEST** 页看编码器、灰度、OpenMV 识别结果和 ADC。
