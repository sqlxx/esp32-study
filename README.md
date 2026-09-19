# esp32-study

经典 ESP32 上的学习固件：BLE 外设、SG90 舵机、BMI088 姿态、开环无刷，以及独立的 AS5600 磁编码器。

需要本机 ESP-IDF v6（本仓库按 v6.0.1 编过）。工程根就是仓库根，`idf.py` 在这里执行。

## 编译烧录

```bash
source ~/.espressif/tools/activate_idf_v6.0.1.sh   # 或你的 export.sh
idf.py build
idf.py -p /dev/tty.usbserial-0001 flash monitor
```

广播名：`ESP32-Hello`。

## 硬件

| 功能 | 引脚 |
| --- | --- |
| SG90 脉宽 | GPIO18（橙/黄信号，红接外部 5V，棕/黑与 ESP32 共地） |
| BMI088 SPI | SCK 14、MOSI 13、MISO 27、CS_ACC 15、CS_GYR 4 |
| 无刷 3PWM | U/V/W = GPIO25/26/32，EN=33，MCPWM group 1 |
| AS5600 I2C | SDA 21、SCL 22（可换脚，不是硬件死脚） |

电机电源不要走 USB，与 ESP32 共地。网页「无刷」页可启停和改转速。「编码器」页看 AS5600 机械角。模块已带 10k 上拉时短线不必再外加。

## Web Bluetooth

Service `0xFFE0`：舵机 `0xFFE1`，IMU Notify `0xFFE2`，无刷 `0xFFE3`，AS5600 Notify `0xFFE4`。

- 电脑 Chrome：`web/` 下 `python3 -m http.server 8080`，打开 `http://localhost:8080`
- Android Chrome：仓库根执行 `./start-web.sh`（或 `web/python3 serve.py`），用提示的 HTTPS 地址
- 不支持 iPhone Safari

舵机 Write：`0~180`、`scan`、`stop`、`spd:1~10`。  
无刷 Write：`on`、`off`、`rpm:30`、`m:0.15`。  
编码器 Notify 8 字节：raw、`deg×10`、`rpm×10`、flags(MD/ML/MH)。

## 目录

```text
main/              BLE + 启动编排
components/servo/  SG90
components/bmi088/ 加速度计 / 陀螺
components/bldc/   开环三相 PWM
components/as5600/ AS5600 磁编码器
web/               Chrome 控制页
```
