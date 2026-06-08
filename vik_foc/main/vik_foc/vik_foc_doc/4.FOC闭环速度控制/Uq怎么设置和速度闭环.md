你这个 `Uq` 不是直接等于 VBUS。**VBUS=12V 时，Uq 绝对不能直接设置到 12V。**

你现在代码里：

```c
#define MOTOR_DRV_VBUS  12.0f
#define UQ_LIMIT        2.2f
```

也就是速度环输出的 `Uq` 最后被限制到 `-2.2V ~ +2.2V`，然后再送进 `vfoc_set_svpwm(uq, M0_TEST_UD, MOTOR_DRV_VBUS)`。

---

## 1. VBUS=12V，Uq 理论最大是多少？

如果你的 `vfoc_set_svpwm()` 是标准电压型 SVPWM，`Uq` 可以理解成 q 轴电压矢量幅值。

理论上线性区大概是：

```text
Uq_max ≈ VBUS / √3
```

所以 12V 母线下：

```text
Uq_max ≈ 12 / 1.732 ≈ 6.9V
```

但是这个是**理论最大线性调制范围**，不是新手调试值。

实际你还有占空比保护：

```c
#define M0_DUTY_MIN 0.02f
#define M0_DUTY_MAX 0.98f
```

说明你的 PWM 不希望打到 0% 或 100%。
所以实际建议别顶到 6.9V，比较稳妥的上限是：

```text
Uq 实用最大值：5V ~ 6V
```

---

## 2. 你现在应该设置多少？

按你现在“没有电流采样、速度环还在调”的阶段，我建议：

### 学习/空载低速阶段

```c
#define UQ_LIMIT  0.8f
```

适合：

```text
30 ~ 100 rpm
```

---

### 低速闭环比较稳定后

```c
#define UQ_LIMIT  1.2f
```

适合：

```text
100 ~ 300 rpm
```

---

### 中速测试

```c
#define UQ_LIMIT  2.0f
```

适合：

```text
300 ~ 1000 rpm
```

---

### 高速测试

```c
#define UQ_LIMIT  3.0f
```

适合慢慢往上试，但前提是：

```text
速度反馈稳定
没有打齿
没有明显发热
没有大电流
没有卡顿
```

你代码注释里其实也写了类似思路：低速先 1~2V，中速 2~3V，高速 3~4V，并且高一点速度建议提高 FOC 控制频率和做斜坡启动。

---

## 3. 最大可以设置多大？

按 12V VBUS 来说，我给你一个明确答案：

```text
新手调试最大：3V 左右
普通空载高速最大：4V 左右
比较激进最大：5V ~ 6V
理论线性极限：约 6.9V
不要设置：12V
```

也就是说：

```c
#define UQ_LIMIT  6.0f
```

已经算很激进了。

如果你没有电流采样，我不建议你直接这么干。因为低速/堵转时，反电动势很小，电流主要由绕组电阻决定：

```text
I ≈ Uq / 相电阻
```

比如你的电机相电阻很低，`Uq = 6V` 可能瞬间电流非常大，MOS、电机、供电都会发热甚至损坏。

所以你现在更合理的是：

```c
#define UQ_LIMIT  1.2f
```

等速度环稳定后再加到：

```c
#define UQ_LIMIT  2.0f
```

---

## 4. 10000RPM 可以吗？

**不要现在直接设置 10000RPM。**

不是说永远不可能，而是你现在这套代码和调试状态不适合。你当前 FOC 控制频率是：

```c
#define M0_FOC_CTRL_FREQ_HZ 1000
```

也就是 1kHz 控制周期。

10000RPM 是：

```text
10000 rpm = 166.7 转/秒
```

如果你的电机是 7 极对数，那么电角频率是：

```text
166.7 × 7 = 1166.7Hz
```

也就是电角度一秒转 1166 个电周期。你现在控制环只有 1000Hz，**连每个电周期采样一次都不够**。这种情况下 FOC 电角度会严重滞后，表现就是：

```text
抖动
尖叫
失步
卡顿
发热
突然停转
```

---

## 5. 想做 10000RPM，至少要满足这些条件

### 条件 1：FOC 控制频率提高

你现在 1kHz 只能适合低速学习。

想上高速，建议至少：

```text
FOC控制频率：10kHz ~ 20kHz
PWM频率：20kHz ~ 40kHz
```

你现在 PWM 是 40kHz，这个可以；但是 FOC 控制频率 1kHz 不够。

---

### 条件 2：速度目标必须斜坡上升

不能这样：

```c
float exp_motor_rpm = 10000.0f;
```

应该这样：

```text
0 -> 500 -> 1000 -> 2000 -> 3000 ... 慢慢升
```

比如：

```c
#define RPM_RAMP_PER_S  500.0f
```

先每秒增加 500rpm。

---

### 条件 3：先验证开环高速能力

你要先做开环测试：

```text
500rpm
1000rpm
1500rpm
2000rpm
3000rpm
```

每一级都看：

```text
电机是否平稳
电流是否过大
MOS是否发热
电机是否发热
速度反馈是否正常
```

如果开环 3000rpm 都不稳，闭环 10000rpm 更不可能稳。

---

### 条件 4：编码器速度反馈要跟得上

如果你用的是 AS5600 这类磁编码器，I2C 读取速度、角度延迟、磁铁安装偏心都会影响高速 FOC。高速下角度延迟会直接变成电角度误差。

比如 10000rpm、7极对数时，电角度转得非常快，角度晚一点点，Uq 就可能不再是正交力矩方向，而是变成抖动/制动方向。

---

## 6. 你现在推荐目标速度怎么设？

按你当前阶段，我建议这样走：

```c
// 第一阶段
exp_motor_rpm = 100.0f;
UQ_LIMIT = 1.2f;

// 第二阶段
exp_motor_rpm = 300.0f;
UQ_LIMIT = 1.5f;

// 第三阶段
exp_motor_rpm = 500.0f;
UQ_LIMIT = 2.0f;

// 第四阶段
exp_motor_rpm = 1000.0f;
UQ_LIMIT = 2.5f ~ 3.0f;
```

等 1000rpm 很稳，再考虑：

```text
2000rpm
3000rpm
5000rpm
```

不要直接跳 10000rpm。

---

## 7. 给你一个实用配置

你现在先这样设置：

```c
#define UQ_LIMIT              1.2f
#define SPEED_UQ_LIMIT        1.2f
#define TARGET_RPM            100.0f
#define RPM_RAMP_PER_S        150.0f
#define UQ_RAMP_PER_S         15.0f
```

稳定后：

```c
#define UQ_LIMIT              2.0f
#define SPEED_UQ_LIMIT        2.0f
#define TARGET_RPM            300.0f
#define RPM_RAMP_PER_S        300.0f
#define UQ_RAMP_PER_S         20.0f
```

再稳定后：

```c
#define UQ_LIMIT              3.0f
#define SPEED_UQ_LIMIT        3.0f
#define TARGET_RPM            1000.0f
#define RPM_RAMP_PER_S        500.0f
#define UQ_RAMP_PER_S         30.0f
```

---

## 最重要结论

**12V VBUS 下，Uq 理论最大约 6.9V，实际建议最大 5~6V；你现在学习调试阶段先别超过 1.2~2.0V。**

**10000RPM 不是只改目标速度就行。你当前 1kHz FOC 控制频率太低，速度反馈也还没完全稳定，不建议直接做 10000RPM。先把 100、300、500、1000RPM 分级调稳，再考虑提高 FOC 控制频率到 10kHz 以上。**
