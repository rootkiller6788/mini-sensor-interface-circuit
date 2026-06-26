# Mini Sensor Interface Circuit（迷你传感器接口电路）

一套**从零开始、零依赖的 C 实现**，涵盖工业、医疗和消费类传感应用的传感器接口电路与信号调理技术。每个子模块对应 MIT、Stanford 和 Berkeley 课程，覆盖从传感器物理到数字化数据的完整模拟前端设计链。

## 子模块

| 子模块 | 主题 | 核心课程 |
|--------|------|----------|
| [mini-4-20ma-current-loop-implementation](mini-4-20ma-current-loop-implementation/) | 4–20 mA 回路合规、2/3/4 线制拓扑、HART 协议、NAMUR NE43 | MIT 2.171, MIT 6.002 |
| [mini-bridge-sensor-strain-gauge-cond](mini-bridge-sensor-strain-gauge-cond/) | 惠斯通电桥、应变系数、1/4/半/全桥、信号调理、ADC 接口 | MIT 6.003, Stanford EE102A |
| [mini-capacitive-sensing-touch-proximity](mini-capacitive-sensing-touch-proximity/) | CDC 拓扑、电荷转移、ΣΔ CDC、kT/C 噪声、手势识别、扩频 | Stanford EE315, MIT 6.775 |
| [mini-instrumentation-amplifier-design](mini-instrumentation-amplifier-design/) | 2/3 运放仪放拓扑、CMRR、失调漂移、噪声建模、增益带宽积、校准 | MIT 6.002, MIT 6.776 |
| [mini-isolated-sensor-digital-isolator](mini-isolated-sensor-digital-isolator/) | 电气隔离、共模瞬态抗扰度、容/磁/光隔离、隔离 SPI/ADC、IEC 60747-5-5 | MIT 6.012, Stanford EE314A |
| [mini-mems-accelerometer-gyro-pcb](mini-mems-accelerometer-gyro-pcb/) | MEMS 加速度计、科里奥利陀螺仪、SPI/I2C 接口、6 轴校准、PCB 设计 | MIT 6.777, Stanford ME310 |
| [mini-photodiode-transimpedance-tia-design](mini-photodiode-transimpedance-tia-design/) | TIA 增益、GBWP、NEP/D*、增益峰化、稳定性补偿、暗/噪声电流 | MIT 6.776, Stanford EE214B |
| [mini-thermocouple-cold-junction-rtd](mini-thermocouple-cold-junction-rtd/) | 塞贝克效应、ITS-90、Callendar–Van Dusen、冷端补偿、4 线 RTD、有理多项式拟合 | MIT 2.171, Stanford ME220 |

## 设计理念

- **零外部依赖** — 纯 C11，仅依赖 `libm` 和标准库
- **自包含子模块** — 每个子模块拥有独立的 `Makefile`、`include/`、`src/`、`tests/`、`examples/`
- **理论到代码的映射** — 每个头文件标注知识覆盖层级（L1–L6），与大学课程对齐
- **模式库** — 建立所有下游电子模块复用的传感器接口编码规范

## 构建

每个子模块独立构建。使用 `make` 构建：

```bash
cd mini-4-20ma-current-loop-implementation
make
make test
```

需要 **GCC**（或任意 C11 编译器）和 **GNU Make**。

## 项目结构

```
28. mini-sensor-interface-circuit/
├── mini-4-20ma-current-loop-implementation/   # 4–20 mA 电流回路、HART 协议、ISA-50.1
├── mini-bridge-sensor-strain-gauge-cond/      # 惠斯通电桥、应变片信号调理
├── mini-capacitive-sensing-touch-proximity/   # 电容测量、触摸/接近传感
├── mini-instrumentation-amplifier-design/     # 仪表放大器拓扑、CMRR 优化
├── mini-isolated-sensor-digital-isolator/     # 电气隔离、隔离式 ADC 接口
├── mini-mems-accelerometer-gyro-pcb/          # MEMS 加速度计/陀螺仪接口、IMU 校准
├── mini-photodiode-transimpedance-tia-design/ # 跨阻放大器设计、光电二极管噪声分析
├── mini-thermocouple-cold-junction-rtd/       # 冷端补偿、RTD 线性化
├── .gitignore
├── README.md
└── README-CN.md
```

## 开源许可

MIT
