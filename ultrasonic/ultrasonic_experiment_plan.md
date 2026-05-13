# 超声波测距模块对比实验项目规划

> **实验目标**：系统比较 A02YYUW（UART 防水型）与 DFRobot URM09（I²C 温度补偿型）两款超声波测距模块在精度、稳定性、抗干扰性、角度响应等维度的性能差异，输出可视化对比报告。
>
> **开发平台**：ESP32-S3 + ESP-IDF v5.x
>
> **文档用途**：可直接作为 AI 辅助编程的上下文提示词，或作为实验执行手册使用。

---

## 一、背景知识

### 1.1 超声波测距原理

超声波传感器通过「发射声波 → 接收回声 → 计算飞行时间」推算目标距离：

```
Distance = (SoundSpeed × TimeOfFlight) / 2
```

- 声速约 340 m/s（受温度影响，每升高 1°C 约增加 0.6 m/s）
- 除以 2 是因为声波需经历「发射→目标→返回」两段路程
- 优质模块会内置温度传感器做实时声速修正（温度补偿）

**双探头设计**：发射（TX）与接收（RX）分离，避免发射信号直接串扰接收端，从而将盲区压缩至 2~3 cm。

### 1.2 两模块核心参数对比

| 参数 | A02YYUW | DFRobot URM09 (SEN0304) |
|---|---|---|
| 通信接口 | **UART 9600bps** | **I²C 地址 0x11** |
| 量程 | 30 ~ 4500 mm | 20 ~ 5000 mm |
| 分辨率 | **1 mm** | 1 cm |
| 盲区 | **3 cm** | ~2 cm |
| 防水等级 | ✅ 防水探头 | ❌ 室内型 |
| 温度补偿 | ❌ | ✅ 内置 |
| 供电电压 | 3.3 V ~ 5.5 V | 3.3 V ~ 5 V |
| 参考价格 | ¥65 | ¥89 |

### 1.3 通信协议详解

**A02YYUW — UART 被动输出**

模块以约 10 Hz 频率持续输出 4 字节帧，主控只需接收：

```
帧格式：[0xFF] [H] [L] [SUM]
距离(mm) = (H << 8) | L
校验和   = (0xFF + H + L) & 0xFF
有效范围：30 ~ 4500 mm，返回 0 表示超出量程
```

**URM09 — I²C 寄存器读写（V3.0 版本，经硬件验证）**

主控主动触发，寄存器映射如下：

| 寄存器 | 地址 | 说明 |
|---|---|---|
| CMD | 0x08 | 写 0x01 触发一次测量 |
| DIST_H | 0x03 | 距离高字节 (cm) |
| DIST_L | 0x04 | 距离低字节 (cm) |
| TEMP_H | 0x05 | 温度高字节 (×0.1°C, 有符号) |
| TEMP_L | 0x06 | 温度低字节 |

> **注意**：URM09 V3.0 版本的寄存器地址与早期版本不同。CMD 寄存器为 0x08（非 0x01），量程配置寄存器 0x11 在新版本中已无效。

触发后需等待约 **120 ms** 再读取结果。

---

## 二、硬件准备

### 2.1 物料清单

| 编号 | 器件 | 数量 | 备注 |
|---|---|---|---|
| 1 | ESP32-S3 开发板 | 1 | 已配置 IDF 环境 |
| 2 | A02YYUW 超声波模块 | 1 | UART，4线（VCC/GND/TX/RX） |
| 3 | DFRobot URM09 (SEN0304) | 1 | I²C，4线（VCC/GND/SDA/SCL） |
| 4 | 4.7 kΩ 电阻 | 2 | I²C 上拉（URM09 板载可能已有） |
| 5 | 面包板 + 杜邦线 | 若干 | — |
| 6 | USB 数据线 | 1 | 烧录 + 串口监视 |
| 7 | 钢卷尺 / 激光尺 | 1 | 标定参考距离 |
| 8 | 平整反射板（A4 纸/木板） | 1 | 作为测距目标物 |

### 2.2 接线方案

**A02YYUW → ESP32-S3**

```
A02YYUW 红线 (VCC)      →  3.3V
A02YYUW 黑线 (GND)      →  GND
A02YYUW 绿线/黄线 (TX)  →  GPIO18  ← UART1 RXD（传感器发送→ESP32接收）
A02YYUW 白线 (RX)       →  GPIO17  ← UART1 TXD（保持 HIGH=稳定模式）
```

**URM09 → ESP32-S3**

```
URM09 VCC  →  3.3V
URM09 GND  →  GND
URM09 SDA  →  GPIO8   ← I2C0 SDA（外接 4.7kΩ 上拉至 3.3V）
URM09 SCL  →  GPIO9   ← I2C0 SCL（外接 4.7kΩ 上拉至 3.3V）
```

> ⚠️ **注意**：两模块可同时连接，UART 与 I²C 总线互不干扰。

---

## 三、软件架构

### 3.1 项目目录结构

```
ultrasonic_compare/
├── CMakeLists.txt              # 根构建文件
└── main/
    ├── CMakeLists.txt          # 组件注册
    ├── main.c                  # 主程序：测试调度 + JSON 输出
    ├── a02yyuw.h / a02yyuw.c   # A02YYUW UART 驱动
    └── urm09.h   / urm09.c     # URM09 I²C 驱动
```

配套 PC 端工具：

```
visualize.py    # Python 可视化脚本（实时 / 离线 / 演示三种模式）
```

### 3.2 驱动接口规范

**A02YYUW 驱动**

```c
// 初始化 UART1，9600bps，RX=GPIO18，TX=GPIO17
esp_err_t a02yyuw_init(int rx_pin, int tx_pin);

// 同步读取一帧：flush旧数据 → 等100ms → 批量读取 → 扫描0xFF帧头取最新帧
// result->distance_mm：有效距离（mm）；result->valid：帧有效标志
esp_err_t a02yyuw_read(a02yyuw_result_t *result);

void a02yyuw_deinit(void);
```

**URM09 驱动**（使用 IDF v6.x 新版 i2c_master API，V3.0 寄存器映射）

```c
// 初始化 I2C0 总线 + 添加 0x11 设备（V3.0 版本）
esp_err_t urm09_init(i2c_master_bus_handle_t *bus, i2c_master_dev_handle_t *dev,
                     int sda_pin, int scl_pin);

// 触发测量（写 CMD=0x08 寄存器 0x01）→ 等待 120ms → 读距离(0x03)+温度(0x05)
// result->distance_cm / result->temperature / result->valid
// 异常值自动过滤：0xFFFF 和超量程值
esp_err_t urm09_read(i2c_master_dev_handle_t dev, urm09_result_t *result);

void urm09_deinit(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t dev);
```

### 3.3 串口输出数据格式

所有采样数据以 **JSON 行**形式从 USB-UART 输出（115200 bps），供 PC 端实时解析：

```json
// 采样数据行
{"t":12345,"ph":1,"i":5,"a":{"d":1023,"ok":true},"u":{"d":102,"ok":true,"temp":24.5}}

// 阶段统计行（每阶段结束后输出）
{"event":"phase_stats","phase":1,"name":"Static_100cm",
 "a":{"mean":10230,"std":82,"min":10180,"max":10290,"err":0},
 "u":{"mean":102,"std":2.1,"min":100,"max":104,"err":1}}

// 事件行
{"event":"phase_start","phase":0,"name":"Static_20cm","samples":30}
{"event":"test_complete"}
```

字段说明：

| 字段 | 含义 |
|---|---|
| `t` | 启动后毫秒时间戳 |
| `ph` | 测试阶段编号（-1=连续监测） |
| `i` | 阶段内采样序号 |
| `a.d` | A02YYUW 距离，单位 **mm** |
| `u.d` | URM09 距离，单位 **cm** |
| `u.temp` | URM09 内置温度，单位 °C |
| `*.ok` | 本次读取是否有效 |

---

## 四、实验方案设计

### 4.1 测试阶段总览

| 阶段 | 编号 | 名称 | 目标距离 | 采样数 | 考察指标 |
|---|---|---|---|---|---|
| 静态近距 | 0 | Static_20cm | 20 cm | 30 | 盲区、近距精度 |
| 静态中距 | 1 | Static_100cm | 100 cm | 30 | 常用距离精度 |
| 静态远距 | 2 | Static_300cm | 300 cm | 30 | 远距衰减、精度退化 |
| 角度 0° | 3 | Angle_0deg | 20 cm | 30 | 基准正对 |
| 角度 30° | 4 | Angle_30deg | 20 cm | 30 | 轻度偏转 |
| 角度 45° | 5 | Angle_45deg | 20 cm | 30 | 边缘波束角 |
| 稳定性 | 6 | Stability_100x | 50 cm | 100 | 噪声分布、漂移 |

### 4.2 各阶段操作规程

#### 阶段 0 — 静态近距（Static_20cm）

1. 将反射板竖直放置，用卷尺量出距两探头 **20.0 cm** 处做标记
2. 两模块探头正对反射板，水平放置，保持静止
3. 等待串口提示 `phase_start`，程序自动采集 30 次
4. 采集完成后输出统计数据摘要，**程序暂停等待按 ENTER**
5. 调整好下一阶段设置后按 ENTER 继续

#### 阶段 1 — 静态中距（Static_100cm）

1. 反射板移至 **100.0 cm** 处
2. 确保测试环境无运动物体干扰
3. 自动采集 30 次；此距离为两模块标称最佳工作区间
4. 采集完成后输出统计数据摘要，**程序暂停等待按 ENTER**

#### 阶段 2 — 静态远距（Static_300cm）

1. 反射板移至 **300.0 cm** 处（URM09 需提前切换至 500cm 量程，程序已自动配置）
2. 确保走廊或空旷环境，无侧面反射干扰
3. 观察 A02YYUW（1mm 分辨率）与 URM09（1cm 分辨率）的噪声量级差异
4. 采集完成后输出统计数据摘要，**程序暂停等待按 ENTER**

#### 阶段 3~5 — 角度测试（Angle_0/30/45deg）

1. 反射板固定于 **20 cm**，逐步将探头相对反射板法线偏转 0°、30°、45°
2. 使用量角器确认角度；探头位置保持不动，转动反射板
3. 重点观察：哪个模块在大角度时最先出现大量无效读数（超出回波接收角）
4. 每个角度阶段完成后**程序暂停等待按 ENTER**，方便调整角度

#### 阶段 6 — 稳定性（Stability_100x）

1. 反射板固定于 **50 cm** 正对，绝对静置
2. 连续采集 **100 次**，计算均值/标准差/最大偏差
3. 输出距离分布直方图，评估读数噪声形态（高斯？还是多峰？）

### 4.3 对比实验控制变量

| 控制项 | 要求 |
|---|---|
| 同一目标物 | 两模块对同一反射板测量，换模块不换位置 |
| 同步时间窗 | 同一阶段内轮流采样（先 A02，再 URM09，间隔 50ms） |
| 温度一致 | 全程在同一室内环境完成，记录环境温度 |
| 供电稳定 | 使用开发板 3.3V 引脚供电，避免外部电源波动 |
| 光照 | A02YYUW 宣称不受光照影响，可补充一组强光/弱光对比 |

---

## 五、可视化报告

### 5.1 Python 脚本使用方法

**安装依赖**

```bash
pip install matplotlib pyserial numpy pandas
```

**三种运行模式**

```bash
# 1. 实时模式：串口连接 ESP32，边采样边绘图
python3 visualize.py --live --port /dev/ttyUSB0

# Windows 示例
python3 visualize.py --live --port COM3

# 2. 离线分析：先保存日志，再分析
idf.py monitor | tee sensor_log.txt        # 采集时同步保存
python3 visualize.py --file sensor_log.txt  # 事后分析

# 3. 演示模式：不需要硬件，验证图表样式
python3 visualize.py --demo
```

### 5.2 生成图表说明

报告图共 **6 张子图**，布局如下：

```
┌─────────────────────────────┬──────────────┐
│  [1] 各阶段均值柱状图         │ [2] 标准差    │
│      (分组对比 + 误差棒)      │    水平条形图  │
├─────────────────────────────┴──────────────┤
│  [3] 全程时序折线图（含阶段背景色区分）       │
├──────────────┬──────────────┬──────────────┤
│ [4] 读取失败率│ [5] 稳定性   │ [6] 两传感器  │
│    柱状图    │    分布直方图  │    散点相关图  │
└──────────────┴──────────────┴──────────────┘
```

| 图编号 | 图表类型 | 横轴 | 纵轴 | 解读要点 |
|---|---|---|---|---|
| 1 | 分组柱状图 | 测试阶段 | 距离 (cm) | 两模块均值接近说明测量一致；误差棒越短越稳定 |
| 2 | 水平条形图 | 标准差 (cm) | 测试阶段 | 越短越好；重点看角度阶段差异 |
| 3 | 双折线图 | 时间 (s) | 距离 (cm) | 直观看漂移、毛刺、异常值 |
| 4 | 分组柱状图 | 测试阶段 | 失败率 (%) | 反映通信可靠性；UART 易受电气干扰 |
| 5 | 直方图 | 距离 (cm) | 密度 | 分布越集中（窄峰）越精确；σ 值直接标注 |
| 6 | 散点图 | URM09 (cm) | A02YYUW (cm) | Pearson r 越接近 1 说明两者测值越一致 |

---

## 六、预期结果与分析指引

### 6.1 预期性能差异

| 测试维度 | 预期优势方 | 原因分析 |
|---|---|---|
| 分辨率/近距精度 | **A02YYUW** | 1mm vs 1cm 硬件分辨率 |
| 远距稳定性（温变环境） | **URM09** | 内置温度补偿修正声速 |
| 通信可靠性 | **URM09** | I²C 应答机制 vs UART 单向广播 |
| 大角度测量（45°） | 待验证 | 取决于各自波束角宽度 |
| 抗强光干扰 | **A02YYUW** | 产品页明确标注不受光照影响 |
| 综合性价比 | **A02YYUW** | ¥65 vs ¥89，防水加分 |

### 6.2 数据异常排查

| 现象 | 可能原因 | 排查方法 |
|---|---|---|
| A02YYUW 持续返回 -1 | UART RX 接错引脚 / 波特率不对 | 用示波器或逻辑分析仪抓 GPIO17 波形 |
| URM09 I²C 初始化失败 | 上拉电阻缺失 / 地址冲突 | 运行 `i2c_scan` 扫描总线确认 0x11 存在 |
| 两模块读数相差 >5 cm | 探头未对齐 / 一模块角度偏移 | 用直角尺确认两模块探头平行且正对目标 |
| 稳定性测试出现双峰分布 | 环境中有第二反射面（墙/桌边） | 清空测试区域，确保探头 ±30° 范围内无杂物 |
| URM09 温度读数异常 | 寄存器字节序理解有误 | 温度为有符号 int16，单位 0.1°C，需强制转换 |

---

## 七、构建与运行流程

### 7.1 ESP-IDF 构建命令

```bash
# 进入项目目录
cd ultrasonic_compare

# 设置目标芯片
idf.py set-target esp32s3

# 构建
idf.py build

# 烧录并打开串口监视（Ctrl+] 退出）
idf.py flash monitor

# 仅保存日志（另开终端）
idf.py monitor | tee sensor_log.txt
```

### 7.2 IDF 版本兼容说明

本项目使用 **ESP-IDF v5.x 新版 I²C API**（`i2c_master_bus_*`）。

如使用 **IDF v4.x**，需将 `urm09.c` 中的 I²C 操作替换为旧版 API：

```c
// v4.x 替代写法
i2c_master_write_to_device(I2C_NUM_0, URM09_I2C_ADDR, buf, len, timeout);
i2c_master_read_from_device(I2C_NUM_0, URM09_I2C_ADDR, data, len, timeout);
```

### 7.3 快速验证（上电后预期串口输出）

```
I (312) MAIN: === Ultrasonic Sensor Comparison Test ===
I (318) A02YYUW: A02YYUW OK — UART1 RX=GPIO18 TX=GPIO17
I (325) URM09: URM09  OK — I2C0 SDA=GPIO8 SCL=GPIO9 addr=0x11
{"event":"phase_start","phase":0,"name":"Static_20cm","samples":30}
{"t":3201,"ph":0,"i":0,"a":{"d":198,"ok":true},"u":{"d":20,"ok":true,"temp":24.3}}
{"t":3551,"ph":0,"i":1,"a":{"d":201,"ok":true},"u":{"d":20,"ok":true,"temp":24.3}}
...
```

---

## 八、扩展实验建议

完成基础对比实验后，可按需增加以下进阶测试：

| 扩展项 | 目的 | 操作要点 |
|---|---|---|
| 温度影响测试 | 验证 URM09 温补效果 | 在 10°C 和 35°C 环境下分别测量同一固定距离，对比两模块误差 |
| 不同材质目标 | 评估回波强度影响 | 分别用布料、泡沫、金属板作为反射目标 |
| 多目标干扰 | 评估抗混响能力 | 在探头侧方 30 cm 处放置第二个物体 |
| 动态目标追踪 | 评估刷新率 | 慢速移动目标（约 5 cm/s），绘制追踪曲线 |
| 长时稳定性 | 评估热漂移 | 连续运行 30 分钟，观察读数随温升的变化趋势 |

---

## 九、参考资料

| 资源 | 链接 |
|---|---|
| DFRobot URM09 官方 Wiki | https://wiki.dfrobot.com.cn/_SKU_SEN0304_URM09_Ultrasonic_Sensor_Gravity_I²C |
| DFRobot URM09 Arduino 驱动 | https://github.com/DFRobot/DFRobot_URM09 |
| ESP-IDF I²C Master 文档 | https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/i2c.html |
| ESP-IDF UART 文档 | https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/uart.html |
| A02YYUW 产品页（淘宝） | 搜索关键词：A02YYUW 超声波防水测距模块 |

---

*文档版本：v1.0 | 生成日期：2026-05-13 | 平台：ESP32-S3 + ESP-IDF v5.x*
