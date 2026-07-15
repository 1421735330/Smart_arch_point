# Smart Place Point

基于 STM32L031 的姿态数据采集固件。系统通过 SPI 读取 ICM-42688-P
加速度计和陀螺仪数据，经 LPUART 发送到 VG6328A-MS BLE 模块，并周期性控制
`PWR_WAKE` 引脚唤醒充电芯片。

## 当前功能

- 使用 STM32L031F6P6 作为主控，工程由 STM32CubeMX 生成并使用 Keil MDK-ARM 构建。
- 初始化 ICM-42688-P，校验 `WHO_AM_I = 0x47`。
- 加速度计配置为 `+/-16 g`、1 kHz，陀螺仪配置为 `+/-2000 dps`、1 kHz。
- 通过 PA4 的数据就绪中断读取 IMU，并提供 10 ms 状态寄存器轮询作为后备。
- 将 VG6328A-MS 配置为 BLE 从机透传模式，串口参数为 115200、8N1。
- 将每帧 IMU 原始数据封装为 17 字节并通过 BLE Notify 发送。
- `PWR_WAKE` 默认保持高电平，每 10 秒输出一次持续 1 秒的低电平脉冲。

## 硬件组成

| 器件 | 型号 | 用途 |
| --- | --- | --- |
| MCU | STM32L031F6P6 | 主控制器 |
| IMU | ICM-42688-P | 温度、三轴加速度和三轴角速度采集 |
| BLE | VG6328A-MS | BLE 从机透传 |
| 充电芯片 | 参见硬件设计 | 由 `PWR_WAKE` 周期性唤醒 |

## 主要引脚

| MCU 引脚 | 信号 | 说明 |
| --- | --- | --- |
| PA0 | `PWR_WAKE` | 充电芯片唤醒，开漏输出 |
| PA1 | `BLE` | BLE 模块状态输入 |
| PA2 / PA3 | `LPUART1_TX/RX` | BLE 模块串口 |
| PA4 | `ICM_INT1` | ICM-42688-P 数据就绪中断，上升沿触发 |
| PA5 / PA6 / PA7 | `SPI1_SCK/MISO/MOSI` | ICM-42688-P SPI 总线 |
| PA9 | `ICM_CS` | ICM-42688-P 片选，低电平有效 |
| PB1 | `GPIO_EXTI1` | 预留外部中断 |
| PA13 / PA14 | `SWDIO/SWCLK` | 下载和调试接口 |

## BLE 数据帧

每帧共 17 字节，多字节数据采用大端序：

| 偏移 | 长度 | 内容 |
| --- | ---: | --- |
| 0 | 1 | 帧头 `0xAA` |
| 1 | 1 | 帧头 `0x55` |
| 2 | 2 | 温度原始值 `int16` |
| 4 | 2 | X 轴加速度原始值 `int16` |
| 6 | 2 | Y 轴加速度原始值 `int16` |
| 8 | 2 | Z 轴加速度原始值 `int16` |
| 10 | 2 | X 轴角速度原始值 `int16` |
| 12 | 2 | Y 轴角速度原始值 `int16` |
| 14 | 2 | Z 轴角速度原始值 `int16` |
| 16 | 1 | 字节 2 至 15 累加和的低 8 位 |

BLE 调试工具连接模块后，需要订阅透传服务中的 Notify 特征才能持续接收数据。

## 构建与下载

1. 安装 STM32CubeMX、Keil MDK-ARM 和 STM32L0 Device Family Pack。
2. 使用 STM32CubeMX 打开 `Hardware/Smart_Place_Point.ioc` 查看外设和引脚配置。
3. 使用 Keil 打开 `Hardware/MDK-ARM/Smart_Place_Point.uvprojx`。
4. 编译工程，并确认下载设置中选择了适用于 STM32L0 的 Flash Algorithm。
5. 通过 SWD 接口下载到目标板。

重新生成 CubeMX 代码前，请先检查 `Core/Src/spi.c`、`Core/Src/usart.c` 和
`Core/Src/main.c` 中的自定义逻辑，生成后应通过 Git diff 确认它们没有被覆盖。

## 常用参数

- 唤醒周期：`Hardware/Core/Src/main.c` 中的 `PWR_WAKE_PERIOD_MS`。
- 低电平持续时间：`Hardware/Core/Src/main.c` 中的 `PWR_WAKE_LOW_TIME_MS`。
- ICM 量程和输出速率：`Hardware/Core/Inc/spi.h` 中的配置宏。
- BLE 超时、启动延时和 Notify 分包长度：`Hardware/Core/Inc/usart.h`。

## 目录结构

```text
Hardware/
|-- Core/               STM32 应用源码与头文件
|-- Drivers/            STM32 HAL 和 CMSIS
|-- MDK-ARM/            Keil 工程配置
|-- Smart_Place_Point.ioc
`-- *.pdf               器件资料
```

第三方 HAL、CMSIS、器件资料和厂商文件仍分别适用其原有许可证及版权声明。
