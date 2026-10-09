# BLDC 控制分层

开环、位置环、速度环、电流环共用同一条电压出口。外环只决定谁去填 `ud`、`uq` 和电气角 `theta_e`，PWM 和反 Park 不随模式改写。

现在没有相电流采样。位置环和速度环先工作在电压模式：速度环的输出直接当作 `uq`。电流采样接上之后，速度环改输出 `iq_ref`，中间插入电流 PI，`bldc_modulate()` 的入参不变。

## 模块

都放在 `components/bldc/`。`as5600` 继续只做传感器，不写控制律。

| 文件 | 职责 | 何时加 |
|---|---|---|
| `bldc_pwm.c` | 占空比 0..1 写进 MCPWM，驱动 EN | 已有 |
| `bldc_foc.c` | `bldc_modulate(ud, uq, theta_e, duty)`：反 Park，再合成三相占空比 | 已接上开环 |
| `bldc_pi.c` | 一个 PI。位置环、速度环各一个实例；Id、Iq 以后再加 | 已接上 |
| `bldc_ctrl.c` | 模式、目标、`bldc_step(dt)` | 开环、位置环、速度环；电流未打开 |
| `bldc.c` | 任务、对外 API（使能、设目标、读状态） | 任务只计时并调用 `bldc_step` |
| `include/bldc.h` | 只给 `main` / GATT 用的接口 | 已有 |

`bldc_foc.h`、`bldc_pi.h`、`bldc_ctrl.h` 放在组件目录里，用现有的 `PRIV_INCLUDE_DIRS "."`，不进 public include。GATT 继续只调 `bldc.h`。

`CMakeLists.txt` 里逐个加上新的 `SRCS`。电流采样以后是另一个文件（例如 `bldc_sense.c`），不塞进 PWM。

## 唯一的电压出口

```c
esp_err_t bldc_modulate(float ud, float uq, float theta_e, float duty_uvw[3]);
```

内部是反 Park，再把 `αβ` 变成 U/V/W 占空比，最后 `bldc_pwm_set_duty()`。`duty_uvw` 可传 `NULL`。开环调用是 `ud = 0`、`uq = modulation`，`theta_e` 用转速积分。`uq > 0` 且角度增加时相序为 U→V→W，转向与原来的三相正弦相同。SVPWM 以后只改这个函数内部。

正 Clarke / 正 Park 先不写。等有 `ia, ib, ic` 再加，输出给电流 PI。

## 一步控制

任务周期仍先用现在的 2 ms。控制律全部放在 `bldc_step(float dt)`，任务只负责计时和调用。以后电流环要提到 PWM 中断或 ADC 中断时，搬的是调用点，不是公式。

每个周期按外面到里面算：

1. 需要编码器时读 AS5600 的机械角。速度用本周期解卷角做差分，不用 `as5600_sample_t.rpm`。位置环把这个差分滤到约 10 ms，速度环再滤到约 200 ms。
2. 位置环开着：机械角误差经 PI 得到 `omega_ref`。否则用外部给的转速目标。
3. 角度从哪来：
   - 开环：`theta_e` 由转速积分，`ud = 0`，`uq = modulation`。
   - 位置环、速度环：`theta_e = 方向 * 极对数 * 编码器机械角 + 零位偏移`（`bldc_ctrl_mech_to_elec`）。方向只乘这一次。
4. 速度环开着：比例项用约 10 ms 的转速误差，积分项用约 200 ms 的平均转速误差，得到扭矩指令。
   - 电压模式：该指令就是 `uq`，`ud = 0`。
   - 电流模式：该指令是 `iq_ref`，`id_ref = 0`，电流 PI 产出 `ud`、`uq`。
5. `bldc_modulate(ud, uq, theta_e)`。

关断只清除运行请求，不直接写 PWM。控制任务在 `bldc_step()` 开头看到请求后，清电气角、把三相占空比写成 0、拉低 EN，不跑积分和调制。占空比与 EN 只由 `bldc_step()` 写；任务创建前的初始化走同一条关断路径。以后有 PI 时，也在这一步清积分。

## 模式

两根轴，避免「位置环输出」在加电流环时改含义却散落在各处。

```c
typedef enum {
    BLDC_MOTION_OPENLOOP = 0, /* 电气角 = ∫ 转速 */
    BLDC_MOTION_POSITION,     /* 位置 PI → 转速 */
    BLDC_MOTION_VELOCITY,     /* 编码器电气角，速度 PI → uq */
} bldc_motion_t;

typedef enum {
    BLDC_TORQUE_VOLTAGE = 0, /* 速度 PI 的输出当作 uq */
    BLDC_TORQUE_CURRENT,     /* 速度 PI 的输出当作 iq_ref */
} bldc_torque_t;
```

带来顺序和开关：

| 阶段 | motion | torque | 实际在跑的环 | 状态 |
|---|---|---|---|---|
| 开环 | `OPENLOOP` | `VOLTAGE` | 无。`uq = modulation`，角度开环积分 | 已接上 |
| 位置 + 速度 | `POSITION` | `VOLTAGE` | 位置 PI → 目标转速，速度 PI → `uq`，电气角来自编码器 | 已接上 |
| 速度 | `VELOCITY` | `VOLTAGE` | 约 200 ms 平均转速经 PI 得到 `uq`，电气角来自编码器 | 已接上 |
| 电流 | 同上 | `CURRENT` | 电流 PI 插在速度 PI 和 `bldc_modulate` 之间 | 未打开 |

位置环的转速用本拍解卷后的计数差分，再经过约 10 ms 滤波。位置环里的速度 PI 靠小 Ki 顶过摩擦死区。积分项单独限在输出范围内。位置环和速度环的 `|uq|` 不超过调制度，最高 0.95。

速度环同样用编码器电气角。2 ms 差分一格编码器就有大约 7 rpm，所以不直接用差分：比例项吃约 10 ms 的滤波转速，抖动约 0.006 的 `uq`，够在每圈的阻力点及时补电压；积分项吃约 200 ms 的平均转速，把平均转速收到设定值。只用 200 ms 平均做反馈时，转子在阻力点停两三百毫秒，环路还来不及反应，转速在一圈里 2～80 rpm 地跳。输出顶到限幅、积分还往同一边走时，积分停止累加。编码器丢帧时转速估计保持原值，恢复后第一拍只重取基准。用手拖慢轴时，`uq` 会往调制度涨，这就是闭环。电流环最后插入，动的是 `bldc_torque_t` 和电流 PI，调制函数不动。

极对数放在 `bldc_ctrl.c`。方向和电气零位也在这里，但第一次进入位置环或速度环时才测量：先用 `ud` 把转子吸到电气角 0，再正转半个电气周期，看编码器计数增减。零位仍是 0 时，`uq` 落在 d 轴上，速度环没有转矩，轴不转；位置环则容易变成正反馈，一直转。

## 对外 API

`bldc_init` / `bldc_enable` / `bldc_disable` 保持。开环阶段可以继续用 `bldc_set_openloop_rpm()` 和 `bldc_set_modulation()`。

后面加模式时再扩 `bldc.h`，例如设运动模式、转矩模式、位置目标、速度目标。状态结构里补上 `ud`、`uq`、`theta_e`，方便 BLE 对照。GATT 不包含 PI 和坐标变换。

## 先不要做的

- 不要在 `bldc_pwm.c` 里做 Park / Clarke。
- 不要让位置环直接去改三相占空比。
- 不要把电流环写成另一套 PWM 路径。
- 电流采样没有之前，不要把速度环的输出定义成 `iq_ref`。
