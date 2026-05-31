可以先从 **`vik_foc.c` 的第 5 步入手**，也就是现在这个地方：

```c
/*
 * 5. SPWM：Ua/Ub/Uc -> duty_Ua/duty_Ub/duty_Uc
 */
vfoc_dt.motor_drv_val.spwm_duty_val =
    vfoc_spwm_calc_duty(&vfoc_dt.motor_drv_val, vbus);
```

你前面这几步先不要动：

```c
Ud/Uq
  ↓
park_inv_transform()
  ↓
Ualpha / Ubeta
  ↓
clark_inv_transform()
  ↓
Ua / Ub / Uc
```

这些仍然是 FOC 主链路。真正从 SPWM 换成 SVPWM，第一步只需要把 **`Ua/Ub/Uc -> duty`** 的算法换掉。你现在的 `m0_foc_control_task()` 只是拿 duty，然后调用 `m0_fd6287_set_duty()` 写入 MCPWM，这一层也可以先不动。 

---

## 1. 你现在的 SPWM 是什么？

你现在的 SPWM duty 计算是：

```c
duty.duty_Ua = 0.5f + motor_v->Ua / vbus;
duty.duty_Ub = 0.5f + motor_v->Ub / vbus;
duty.duty_Uc = 0.5f + motor_v->Uc / vbus;
```

也就是三相电压直接平移到 50% 附近。比如 `Ua = 0V`，输出 `50%`；`Ua = +1V`，占空比高一点；`Ua = -1V`，占空比低一点。

SVPWM 的零序注入法，就是在这之前加一步：

```c
Ua / Ub / Uc
  ↓
找最大值 max
找最小值 min
  ↓
offset = -0.5f * (max + min)
  ↓
Ua += offset
Ub += offset
Uc += offset
  ↓
再转 duty
```

你之前那个 ESP-IDF float 示例里已经有这个思路：先做 Clarke 反变换得到三相，再找 `max/min`，最后给三相一起加公共偏移。

---

## 2. 最小改动方案：新增 `vfoc_svpwm_calc_duty()`

在 `vik_foc.c` 里新增这个函数：

```c
/**
 * @brief SVPWM 零序注入法：Ua/Ub/Uc -> duty_Ua/duty_Ub/duty_Uc
 *
 * 说明：
 * 1. 先拿到逆 Clarke 得到的三相电压 Ua/Ub/Uc
 * 2. 找三相最大值 max 和最小值 min
 * 3. 三相一起加一个公共偏移 offset
 * 4. 再转换成 0.0 ~ 1.0 的 PWM duty
 *
 * 这个 offset 不改变线电压：
 * Uab = Ua - Ub
 * Ubc = Ub - Uc
 * Uca = Uc - Ua
 *
 * 因为三相都加了同一个 offset，所以相互之间的差值不变。
 */
spwm_duty_t vfoc_svpwm_calc_duty(const motor_driver_parm_t *motor_v, float vbus)
{
    spwm_duty_t duty = {0};

    float ua;
    float ub;
    float uc;

    float max_v;
    float min_v;
    float offset;

    if ((motor_v == NULL) || (vbus <= 0.0f))
    {
        return duty;
    }

    /*
     * 1. 先取出三相电压
     */
    ua = motor_v->Ua;
    ub = motor_v->Ub;
    uc = motor_v->Uc;

    /*
     * 2. 找最大值和最小值
     */
    max_v = fmaxf(fmaxf(ua, ub), uc);
    min_v = fminf(fminf(ua, ub), uc);

    /*
     * 3. SVPWM 零序注入
     *
     * 这个 offset 会把三相波形整体上下移动，
     * 让 PWM 更充分利用 0%~100% 的范围。
     */
    offset = -0.5f * (max_v + min_v);

    ua += offset;
    ub += offset;
    uc += offset;

    /*
     * 4. 转换成 duty
     *
     * 注意：这里仍然是 0.5 + 电压 / 母线电压
     * 只是电压已经被 SVPWM offset 修正过。
     */
    duty.duty_Ua = 0.5f + ua / vbus;
    duty.duty_Ub = 0.5f + ub / vbus;
    duty.duty_Uc = 0.5f + uc / vbus;

    /*
     * 5. 限幅，防止超过 0%~100%
     */
    duty.duty_Ua = vfoc_limit(duty.duty_Ua, 0.0f, 1.0f);
    duty.duty_Ub = vfoc_limit(duty.duty_Ub, 0.0f, 1.0f);
    duty.duty_Uc = vfoc_limit(duty.duty_Uc, 0.0f, 1.0f);

    return duty;
}
```

因为你 `vik_foc.c` 已经包含了：

```c
#include <math.h>
```

所以 `fmaxf()`、`fminf()` 可以直接用。

---

## 3. 再新增一个 SVPWM 运行函数

你可以先不要删原来的 `vfoc_open_loop_spwm_run()`，而是新增一个：

```c
void vfoc_open_loop_svpwm_run(float target_rpm, float uq, float vbus, float dt_s)
{
    clark_parm_t l_temp_clark_v = {0};

    if (dt_s <= 0.0f)
    {
        return;
    }

    /*
     * 1. 根据目标转速更新电角度
     */
    vfoc_update_open_loop_angle(target_rpm, dt_s);

    /*
     * 2. 开环电压 FOC：d轴为0，q轴给电压
     */
    vfoc_dt.park_val.Uq = uq;
    vfoc_dt.park_val.Ud = 0.0f;

    /*
     * 3. 逆 Park：Ud/Uq -> Ualpha/Ubeta
     */
    l_temp_clark_v = park_inv_transform(&vfoc_dt);

    /*
     * 4. 逆 Clarke：Ualpha/Ubeta -> Ua/Ub/Uc
     */
    vfoc_dt.motor_drv_val = clark_inv_transform(&l_temp_clark_v);

    /*
     * 5. SVPWM：Ua/Ub/Uc -> duty_Ua/duty_Ub/duty_Uc
     */
    vfoc_dt.motor_drv_val.spwm_duty_val =
        vfoc_svpwm_calc_duty(&vfoc_dt.motor_drv_val, vbus);
}
```

注意：结构体名字现在还叫 `spwm_duty_t`，这个可以暂时不改。它本质只是三相 duty，不一定非得代表 SPWM。

---

## 4. 修改 `vik_foc.h`

在头文件里加两个声明：

```c
spwm_duty_t vfoc_svpwm_calc_duty(const motor_driver_parm_t *motor_v, float vbus);
void vfoc_open_loop_svpwm_run(float target_rpm, float uq, float vbus, float dt_s);
```

后面等你代码稳定了，可以把：

```c
spwm_duty_t
```

改名成更通用的：

```c
pwm_duty_t
```

但现在学习阶段先别大改，避免牵一发动全身。

---

## 5. 修改 `m0_foc_control_task()`

你现在任务里调用的是：

```c
vfoc_open_loop_spwm_run(M0_TEST_RPM,
                        M0_TEST_UQ,
                        MOTOR_DRV_VBUS,
                        M0_FOC_DT_S);
```

改成：

```c
vfoc_open_loop_svpwm_run(M0_TEST_RPM,
                         M0_TEST_UQ,
                         MOTOR_DRV_VBUS,
                         M0_FOC_DT_S);
```

下面这些可以不动：

```c
pwm_duty = vfoc_get_spwm_duty();

m0_fd6287_set_duty(pwm_duty.duty_Ua,
                   pwm_duty.duty_Ub,
                   pwm_duty.duty_Uc);
```

因为你的 `m0_fd6287_set_duty()` 本来就只是把 `0.0 ~ 1.0` 的 duty 转成 MCPWM compare 值，并不关心这个 duty 是 SPWM 算出来的还是 SVPWM 算出来的。

---

## 6. 当前阶段不要先动 MCPWM

你现在 MCPWM 是：

```c
.count_mode = MCPWM_TIMER_COUNT_MODE_UP,
```

也就是边沿对齐 PWM。严格来说，电机控制里更常见的是中心对齐：

```c
.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
```

但是下一步建议你 **先不改 MCPWM**。先只把 SPWM duty 算法换成 SVPWM 零序注入，确认电机能转、方向正常、声音没有变差。等 SVPWM 数学链路确认后，再考虑改中心对齐 PWM。

---

## 7. 你现在的修改路线

按优先级来：

```text
第一步：vik_foc.h
    添加 vfoc_svpwm_calc_duty()
    添加 vfoc_open_loop_svpwm_run()

第二步：vik_foc.c
    新增 vfoc_svpwm_calc_duty()
    新增 vfoc_open_loop_svpwm_run()

第三步：m0_fd6287_pwm.c
    把 vfoc_open_loop_spwm_run()
    改成 vfoc_open_loop_svpwm_run()

第四步：先低压低速测试
    M0_TEST_RPM = 50 ~ 200
    M0_TEST_UQ  = 0.8 ~ 1.5V
```

你现在 `M0_TEST_RPM = 300`、`M0_TEST_UQ = 2.0V` 也能测，但刚换 SVPWM 时我建议先降一点，比如：

```c
#define M0_TEST_RPM    100.0f
#define M0_TEST_UQ     1.0f
```

确认没异常后再慢慢加。

---

## 8. 一个很重要的点

SVPWM 不是把 `park_inv_transform()` 和 `clark_inv_transform()` 全部推翻重写。

你现在这条链路：

```text
Uq/Ud
  -> 逆 Park
  -> Ualpha/Ubeta
  -> 逆 Clarke
  -> Ua/Ub/Uc
  -> duty
```

换 SVPWM 时，前面都保留。你只改最后一段：

```text
SPWM:
Ua/Ub/Uc -> 直接平移成 duty

SVPWM:
Ua/Ub/Uc -> 加零序偏移 -> 再平移成 duty
```

所以你下一步最该改的就是：

```c
vfoc_spwm_calc_duty()
```

旁边新增一个：

```c
vfoc_svpwm_calc_duty()
```

然后把运行函数切过去。这样最稳。

-----
------
-----
可以，**最简单直接的实现方式**：你先不要写六扇区法，直接用 **SVPWM 零序注入法** 替代你现在的 `vfoc_spwm_calc_duty()`。

你的当前链路已经是：

```text
Uq / Ud
  ↓
逆 Park
  ↓
Ualpha / Ubeta
  ↓
逆 Clarke
  ↓
Ua / Ub / Uc
  ↓
SPWM duty
```

现在只把最后一步换掉即可。你原来的 `vfoc_open_loop_spwm_run()` 最后就是调用 `vfoc_spwm_calc_duty()` 把三相电压转成 duty，所以直接替换这个函数最省事。

---

## 直接替换这个函数

把你 `vik_foc.c` 里的这个旧函数：

```c
spwm_duty_t vfoc_spwm_calc_duty(const motor_driver_parm_t *motor_v, float vbus)
```

整个替换成下面这个版本：

```c
/**
 * @brief SVPWM 零序注入法：Ua/Ub/Uc -> duty_Ua/duty_Ub/duty_Uc
 *
 * 注意：
 * 这个函数名字虽然还叫 vfoc_spwm_calc_duty，
 * 但内部已经改成 SVPWM 的零序注入算法了。
 *
 * 优点：
 * 1. 不需要改外部调用代码
 * 2. 不需要改 MCPWM 驱动
 * 3. 只替换这一层即可从 SPWM 切到 SVPWM
 *
 * 原理：
 * 普通 SPWM：
 *      duty = 0.5 + phase_voltage / vbus
 *
 * SVPWM 零序注入：
 *      先找到 Ua/Ub/Uc 里的最大值和最小值
 *      offset = -0.5 * (max + min)
 *      三相同时加 offset
 *      再转换成 duty
 *
 * 三相同时加同一个 offset，不会改变线电压：
 *      Uab = Ua - Ub
 *      Ubc = Ub - Uc
 *      Uca = Uc - Ua
 *
 * 但是可以让 PWM 波形更好地利用母线电压。
 */
spwm_duty_t vfoc_spwm_calc_duty(const motor_driver_parm_t *motor_v, float vbus)
{
    spwm_duty_t duty = {0};

    float ua;
    float ub;
    float uc;

    float max_v;
    float min_v;
    float offset;

    if ((motor_v == NULL) || (vbus <= 0.0f))
    {
        return duty;
    }

    /*
     * 1. 取出逆 Clarke 之后的三相电压
     */
    ua = motor_v->Ua;
    ub = motor_v->Ub;
    uc = motor_v->Uc;

    /*
     * 2. 找三相里面的最大值和最小值
     */
    max_v = fmaxf(fmaxf(ua, ub), uc);
    min_v = fminf(fminf(ua, ub), uc);

    /*
     * 3. 计算 SVPWM 零序偏移量
     *
     * 这个 offset 会同时加到三相上。
     * 三相一起平移，不改变相与相之间的电压差，
     * 但是可以提高直流母线电压利用率。
     */
    offset = -0.5f * (max_v + min_v);

    /*
     * 4. 三相同时加入零序分量
     */
    ua += offset;
    ub += offset;
    uc += offset;

    /*
     * 5. 转成 PWM 占空比
     *
     * 你的系统里：
     * 0.5 表示中点，也就是 50% 占空比。
     * 正电压让 duty 大于 50%。
     * 负电压让 duty 小于 50%。
     */
    duty.duty_Ua = 0.5f + ua / vbus;
    duty.duty_Ub = 0.5f + ub / vbus;
    duty.duty_Uc = 0.5f + uc / vbus;

    /*
     * 6. 限幅到 0.0 ~ 1.0
     *
     * 后面 m0_fd6287_set_duty() 里面还会限制到 0.02 ~ 0.98，
     * 这里先限制到数学上的合法范围。
     */
    duty.duty_Ua = vfoc_limit(duty.duty_Ua, 0.0f, 1.0f);
    duty.duty_Ub = vfoc_limit(duty.duty_Ub, 0.0f, 1.0f);
    duty.duty_Uc = vfoc_limit(duty.duty_Uc, 0.0f, 1.0f);

    return duty;
}
```

你之前的 ESP-IDF float 版本示例里，其实也是这个思路：先得到三相 `u/v/w`，然后找最大值、最小值，再加公共零序偏移。

---

## 其他地方先不用改

你现在 `m0_foc_control_task()` 里面还是这样：

```c
vfoc_open_loop_spwm_run(M0_TEST_RPM,
                        M0_TEST_UQ,
                        MOTOR_DRV_VBUS,
                        M0_FOC_DT_S);

pwm_duty = vfoc_get_spwm_duty();

m0_fd6287_set_duty(pwm_duty.duty_Ua,
                   pwm_duty.duty_Ub,
                   pwm_duty.duty_Uc);
```

这里可以先不动。因为你已经把 `vfoc_spwm_calc_duty()` 的内部算法换成 SVPWM 了，外部调用不用改。你的 `m0_fd6287_set_duty()` 本质只负责把 `0.0 ~ 1.0` duty 写到 MCPWM compare，不关心这个 duty 是 SPWM 还是 SVPWM 算出来的。

---

## 现在 SVPWM 的完整过程就是这样

```text
1. vfoc_update_open_loop_angle()
   更新电角度 theta_e

2. 设置：
   Ud = 0
   Uq = 目标 q 轴电压

3. park_inv_transform()
   Ud/Uq + theta_e
   转成 Ualpha/Ubeta

4. clark_inv_transform()
   Ualpha/Ubeta
   转成 Ua/Ub/Uc

5. vfoc_spwm_calc_duty()
   现在函数内部已经是 SVPWM：
   Ua/Ub/Uc
   找 max/min
   加 offset
   转 duty

6. m0_fd6287_set_duty()
   把 duty 写到 MCPWM
```

---

## 建议测试参数先降一点

刚换 SVPWM 时，先这样测：

```c
#define M0_TEST_RPM                100.0f
#define M0_TEST_UQ                 1.0f
#define MOTOR_DRV_VBUS             12.0f
```

能稳定转以后再加：

```c
#define M0_TEST_RPM                300.0f
#define M0_TEST_UQ                 2.0f
```

你现在代码里 `M0_TEST_UQ` 是 2.0V，`MOTOR_DRV_VBUS` 是 12V，这个空载一般可以，但刚换算法最好先保守一点。

---

## 后面更规范的命名

现在为了最小改动，我让你保留了函数名：

```c
vfoc_spwm_calc_duty()
```

但它内部其实已经是 SVPWM 了。

等你跑通后，可以再改名成：

```c
vfoc_svpwm_calc_duty()
```

然后把 `vfoc_open_loop_spwm_run()` 改成：

```c
vfoc_open_loop_svpwm_run()
```

但这一步不是必须。你现在先替换这个函数，就能开始验证 SVPWM。

-----
-----
----
明白。那我不再给你“能跑就行”的版本。

**量产级 SVPWM 核心建议用这个方案：**

```text
Ud/Uq
  ↓
逆 Park
  ↓
Ualpha/Ubeta
  ↓
SVPWM 直接由 alpha/beta 算三相 duty
  ↓
MCPWM 输出
```

也就是说，**量产写法不建议先算 `Ua/Ub/Uc` 再 SPWM 化**，而是直接用 `Ualpha/Ubeta` 进入 SVPWM 调制器。你现在 `park_inv_transform()` 已经能得到 `I_alpha/I_beta`，所以正好可以接 SVPWM。你当前代码里是先逆 Clarke 得到 `Ua/Ub/Uc`，再调用 `vfoc_spwm_calc_duty()`，这个位置就是要替换的地方。

下面这版包含：

```text
1. alpha/beta 直接进入 SVPWM
2. 母线电压 vbus 检查
3. NaN/Inf 检查
4. 线性调制区限幅，防止过调制
5. duty_min / duty_max 保护，避免 0% / 100%
6. 零序注入
7. 返回状态码，方便以后做故障诊断
```

---

# 1. 先改 `vik_foc.h`

在你的 `vik_foc.h` 里面加这些宏和声明。

```c
#ifndef VIK_FOC_H
#define VIK_FOC_H

#include <stdint.h>

/* 根号3 */
#define SQRT3                  1.7320508075688772f
#define FOC_PI                 3.14159265358979323846f
#define FOC_2PI                6.28318530717958647692f

#define FOC_SQRT3_DIV_2        0.8660254037844386f
#define FOC_2_DIV_SQRT3        1.1547005383792515f

/* 角度 → 弧度 */
#define FOC_DEG_TO_RAD(angle)  ((angle) * FOC_PI / 180.0f)

/*
 * 量产级 PWM 占空比保护。
 *
 * FD6287 这类 bootstrap 高边驱动，不建议长时间 0% 或 100%。
 * 你 m0_fd6287_pwm.c 里原本也是 0.02 ~ 0.98。
 */
#define VFOC_PWM_DUTY_MIN      0.02f
#define VFOC_PWM_DUTY_MAX      0.98f

/*
 * 浮点计算保护阈值。
 */
#define VFOC_FLOAT_EPSILON     1.0e-6f


typedef struct
{
    float I_alpha;
    float I_beta;
} clark_parm_t;


typedef struct
{
    float Ud;
    float Uq;
} park_parm_t;


typedef struct 
{
    float duty_Ua;
    float duty_Ub;
    float duty_Uc;
} spwm_duty_t;


typedef struct
{
    float Ua;
    float Ub;
    float Uc;
    spwm_duty_t spwm_duty_val;
} motor_driver_parm_t;


typedef struct
{
    float theta_e;
    unsigned int pole_pairs;
    float theta_m;
} motor_parm_t;


typedef struct 
{
    clark_parm_t clark_val;
    park_parm_t park_val;
    motor_driver_parm_t motor_drv_val;
    motor_parm_t motor_par;
} foc_data_t;


/**
 * @brief SVPWM 运行状态
 */
typedef enum
{
    VFOC_STATUS_OK = 0,

    /* 输入参数有问题 */
    VFOC_STATUS_NULL_PTR,

    /* vbus 不合法 */
    VFOC_STATUS_BAD_VBUS,

    /* alpha/beta 里出现 NaN 或 Inf */
    VFOC_STATUS_BAD_INPUT,

    /*
     * 电压矢量超过线性调制区，已经被缩放。
     * 这不是严重错误，表示算法做了限幅保护。
     */
    VFOC_STATUS_SATURATED,

} vfoc_status_t;


void vfoc_init(void);
void vfoc_update_open_loop_angle(float target_rpm, float dt_s);

motor_driver_parm_t clark_inv_transform(const clark_parm_t *c_v);
clark_parm_t park_inv_transform(const foc_data_t *foc_v);

/*
 * 保留旧 SPWM 接口，方便你对比测试。
 */
spwm_duty_t vfoc_spwm_calc_duty(const motor_driver_parm_t *motor_v, float vbus);

/*
 * 量产级 SVPWM 核心接口：
 * alpha/beta + vbus -> duty
 */
vfoc_status_t vfoc_svpwm_calc_duty_ab(const clark_parm_t *ab_v,
                                      float vbus,
                                      spwm_duty_t *duty_out);

/*
 * 开环 SVPWM 运行接口。
 */
void vfoc_open_loop_svpwm_run(float target_rpm,
                              float uq,
                              float vbus,
                              float dt_s);

void vfoc_open_loop_spwm_run(float target_rpm, float uq, float vbus, float dt_s);

spwm_duty_t vfoc_get_spwm_duty(void);

#endif
```

---

# 2. 在 `vik_foc.c` 里加入量产级 SVPWM 核心

把下面代码放到你的 `vfoc_limit()` 后面。

```c
/**
 * @brief 判断浮点数是否合法
 *
 * 量产代码里，不建议让 NaN/Inf 继续进入 PWM。
 * 一旦 NaN 进入 duty，后面 compare 值可能异常。
 */
static int vfoc_float_is_valid(float value)
{
    return isfinite(value);
}


/**
 * @brief 量产级 SVPWM：alpha/beta -> 三相 duty
 *
 * 输入：
 *      ab_v->I_alpha : alpha 轴电压，单位 V
 *      ab_v->I_beta  : beta 轴电压，单位 V
 *      vbus          : 母线电压，单位 V
 *
 * 输出：
 *      duty_out->duty_Ua : U 相 duty，范围 VFOC_PWM_DUTY_MIN ~ VFOC_PWM_DUTY_MAX
 *      duty_out->duty_Ub : V 相 duty
 *      duty_out->duty_Uc : W 相 duty
 *
 * 核心思想：
 *      1. 对 alpha/beta 电压矢量做线性调制区限幅
 *      2. alpha/beta -> 三相相电压 ua/ub/uc
 *      3. 找 max/min
 *      4. 注入零序 offset = -0.5 * (max + min)
 *      5. 转换为 duty
 *
 * 为什么这是工程版：
 *      - 不依赖扇区判断，避免扇区边界抖动 bug
 *      - 直接 alpha/beta 输入，减少中间层
 *      - 自动限制在线性调制区
 *      - 保留 duty 上下限，保护 bootstrap 驱动
 *      - 返回状态码，方便后续故障记录
 */
vfoc_status_t vfoc_svpwm_calc_duty_ab(const clark_parm_t *ab_v,
                                      float vbus,
                                      spwm_duty_t *duty_out)
{
    float alpha;
    float beta;

    float vref;
    float vref_max;
    float scale;

    float ua;
    float ub;
    float uc;

    float max_v;
    float min_v;
    float zero_offset;

    float duty_half_range;

    vfoc_status_t status = VFOC_STATUS_OK;

    /*
     * 先给安全默认值。
     * 如果后面参数错误，至少输出 50% 附近，不会随机乱跳。
     */
    spwm_duty_t duty = {
        .duty_Ua = 0.5f,
        .duty_Ub = 0.5f,
        .duty_Uc = 0.5f,
    };

    if (duty_out == NULL)
    {
        return VFOC_STATUS_NULL_PTR;
    }

    *duty_out = duty;

    if (ab_v == NULL)
    {
        return VFOC_STATUS_NULL_PTR;
    }

    if ((vbus <= VFOC_FLOAT_EPSILON) || (!vfoc_float_is_valid(vbus)))
    {
        return VFOC_STATUS_BAD_VBUS;
    }

    alpha = ab_v->I_alpha;
    beta  = ab_v->I_beta;

    if ((!vfoc_float_is_valid(alpha)) || (!vfoc_float_is_valid(beta)))
    {
        return VFOC_STATUS_BAD_INPUT;
    }

    /*
     * duty 可用半范围。
     *
     * 理想 duty 是 0.0 ~ 1.0，则半范围是 0.5。
     * 但 FD6287 bootstrap 驱动不建议靠近 0% / 100%，
     * 所以这里用 0.02 ~ 0.98，半范围就是 0.48。
     */
    duty_half_range = 0.5f - VFOC_PWM_DUTY_MIN;

    if ((VFOC_PWM_DUTY_MAX - 0.5f) < duty_half_range)
    {
        duty_half_range = VFOC_PWM_DUTY_MAX - 0.5f;
    }

    if (duty_half_range <= 0.0f)
    {
        return VFOC_STATUS_BAD_INPUT;
    }

    /*
     * SVPWM 线性调制区最大 alpha/beta 电压矢量幅值：
     *
     * 理想情况下：
     *      Vref_max = Vbus / sqrt(3)
     *
     * 考虑 duty_min/duty_max 以后：
     *      Vref_max = 2 / sqrt(3) * duty_half_range * Vbus
     *
     * duty_half_range = 0.5 时：
     *      Vref_max = 0.57735 * Vbus
     *
     * duty_half_range = 0.48 时：
     *      Vref_max ≈ 0.554 * Vbus
     */
    vref = sqrtf((alpha * alpha) + (beta * beta));
    vref_max = FOC_2_DIV_SQRT3 * duty_half_range * vbus;

    /*
     * 超过线性调制区就等比例缩小。
     *
     * 注意：
     * 这里不是粗暴 clamp alpha 或 beta，
     * 而是保持矢量方向不变，只缩小幅值。
     * 这样电压角度不会畸变。
     */
    if ((vref > vref_max) && (vref > VFOC_FLOAT_EPSILON))
    {
        scale = vref_max / vref;
        alpha *= scale;
        beta  *= scale;
        status = VFOC_STATUS_SATURATED;
    }

    /*
     * alpha/beta -> 三相。
     *
     * ua + ub + uc = 0
     */
    ua = alpha;
    ub = (-0.5f * alpha) + (FOC_SQRT3_DIV_2 * beta);
    uc = (-0.5f * alpha) - (FOC_SQRT3_DIV_2 * beta);

    /*
     * 找最大值和最小值。
     */
    max_v = fmaxf(fmaxf(ua, ub), uc);
    min_v = fminf(fminf(ua, ub), uc);

    /*
     * 零序注入。
     *
     * 三相同时加同一个 offset，不改变线电压：
     *      Uab = Ua - Ub
     *      Ubc = Ub - Uc
     *      Uca = Uc - Ua
     *
     * 但是可以把三相波形居中塞进 PWM 可输出范围。
     */
    zero_offset = -0.5f * (max_v + min_v);

    ua += zero_offset;
    ub += zero_offset;
    uc += zero_offset;

    /*
     * 电压 -> duty。
     *
     * duty = 0.5 + phase_voltage / vbus
     */
    duty.duty_Ua = 0.5f + (ua / vbus);
    duty.duty_Ub = 0.5f + (ub / vbus);
    duty.duty_Uc = 0.5f + (uc / vbus);

    /*
     * 最终 duty 保护。
     * 算法前面已经做过线性区缩放，正常不会碰到这里。
     * 这里是最后一道保险。
     */
    duty.duty_Ua = vfoc_limit(duty.duty_Ua, VFOC_PWM_DUTY_MIN, VFOC_PWM_DUTY_MAX);
    duty.duty_Ub = vfoc_limit(duty.duty_Ub, VFOC_PWM_DUTY_MIN, VFOC_PWM_DUTY_MAX);
    duty.duty_Uc = vfoc_limit(duty.duty_Uc, VFOC_PWM_DUTY_MIN, VFOC_PWM_DUTY_MAX);

    *duty_out = duty;

    return status;
}
```

---

# 3. 新增 `vfoc_open_loop_svpwm_run()`

继续放在 `vik_foc.c` 里面，建议放到原来的 `vfoc_open_loop_spwm_run()` 后面。

```c
/**
 * @brief 开环电压 FOC + SVPWM
 *
 * 这是替代 vfoc_open_loop_spwm_run() 的量产风格版本。
 *
 * 注意：
 * 这仍然是开环控制，不是完整量产 FOC。
 * 真正量产还需要：
 *      电流采样
 *      电流环 PI
 *      速度环 PI
 *      位置/速度估算或编码器
 *      过流/过压/欠压/堵转/温度保护
 *
 * 但是这个 SVPWM 调制器本身是工程化写法。
 */
void vfoc_open_loop_svpwm_run(float target_rpm,
                              float uq,
                              float vbus,
                              float dt_s)
{
    clark_parm_t l_temp_clark_v = {0};
    vfoc_status_t svpwm_status;

    if (dt_s <= 0.0f)
    {
        return;
    }

    /*
     * 1. 更新开环电角度。
     */
    vfoc_update_open_loop_angle(target_rpm, dt_s);

    /*
     * 2. 设置 dq 电压。
     *
     * 开环阶段：
     *      Ud = 0
     *      Uq = 给定测试电压
     */
    vfoc_dt.park_val.Ud = 0.0f;
    vfoc_dt.park_val.Uq = uq;

    /*
     * 3. 逆 Park：
     *      Ud/Uq + theta_e -> Ualpha/Ubeta
     */
    l_temp_clark_v = park_inv_transform(&vfoc_dt);

    /*
     * 4. 可选：保存 Ua/Ub/Uc，方便你打印调试。
     *
     * 注意：
     * 真正 SVPWM duty 不依赖这里的 Ua/Ub/Uc。
     * SVPWM 是直接用 alpha/beta 算 duty。
     */
    vfoc_dt.motor_drv_val = clark_inv_transform(&l_temp_clark_v);

    /*
     * 5. SVPWM：
     *      Ualpha/Ubeta -> duty_Ua/duty_Ub/duty_Uc
     */
    svpwm_status = vfoc_svpwm_calc_duty_ab(
                        &l_temp_clark_v,
                        vbus,
                        &vfoc_dt.motor_drv_val.spwm_duty_val
                   );

    /*
     * 量产代码里，不建议 1ms 打一次日志。
     * 这里只在异常时打。
     */
    if ((svpwm_status != VFOC_STATUS_OK) &&
        (svpwm_status != VFOC_STATUS_SATURATED))
    {
        ESP_LOGE(TAG, "SVPWM error, status=%d", (int)svpwm_status);
    }

    /*
     * 如果只是 VFOC_STATUS_SATURATED，说明电压指令过大，
     * 算法已经自动缩放，不需要停机。
     *
     * 后续闭环时，可以把这个状态反馈给电压环/电流环，
     * 做 anti-windup 防积分饱和。
     */
}
```

---

# 4. 修改 `m0_fd6287_pwm.c` 的控制任务

你现在任务里调用的是：

```c
vfoc_open_loop_spwm_run(M0_TEST_RPM,
                        M0_TEST_UQ,
                        MOTOR_DRV_VBUS,
                        M0_FOC_DT_S);
```

改成：

```c
vfoc_open_loop_svpwm_run(M0_TEST_RPM,
                         M0_TEST_UQ,
                         MOTOR_DRV_VBUS,
                         M0_FOC_DT_S);
```

完整替换这一小段：

```c
static void m0_foc_control_task(void *arg)
{
    spwm_duty_t pwm_duty;

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /*
         * 开环 FOC + SVPWM。
         */
        vfoc_open_loop_svpwm_run(M0_TEST_RPM,
                                 M0_TEST_UQ,
                                 MOTOR_DRV_VBUS,
                                 M0_FOC_DT_S);

        /*
         * 获取 SVPWM duty。
         *
         * 这里函数名还叫 vfoc_get_spwm_duty()，
         * 但里面存的已经是 SVPWM duty。
         * 后面可以再统一改名为 vfoc_get_pwm_duty()。
         */
        pwm_duty = vfoc_get_spwm_duty();

        m0_fd6287_set_duty(pwm_duty.duty_Ua,
                           pwm_duty.duty_Ub,
                           pwm_duty.duty_Uc);
    }
}
```

你的 `m0_fd6287_set_duty()` 本来就是把 `0.0 ~ 1.0` 的 duty 转成 MCPWM compare 值，而且里面已经做了 `M0_DUTY_MIN ~ M0_DUTY_MAX` 限幅，所以这一层可以继续保留作为硬件层最后保护。

---

# 5. 为什么这版比六扇区法更适合你现在量产演进？

很多教程会写这种：

```text
判断 sector
计算 T1
计算 T2
计算 T0
根据 sector 分配 Ta/Tb/Tc
```

这种当然是标准 SVPWM 教程写法，但是工程上容易出现几个问题：

```text
1. 扇区边界附近容易因为浮点误差抖动
2. 代码分支多
3. 初学阶段容易把 T1/T2/T0 分配表写错
4. 后面做限幅和 anti-windup 还要额外处理
```

我给你的这个是 **centered zero-sequence SVPWM**：

```text
1. alpha/beta -> ua/ub/uc
2. 找 max/min
3. offset = -0.5 * (max + min)
4. ua/ub/uc 同时加 offset
5. duty = 0.5 + u/vbus
```

它和线性区内的经典 SVPWM 本质等价，而且更适合软件工程实现。你之前 ESP-IDF float 示例里的 `foc_svpwm_zero_sequence_float()` 其实就是这个方向：先反 Clarke 得到三相，再找最大/最小值，然后注入公共偏移。

---

# 6. 但是要注意：SVPWM 代码量产 ≠ 整个电机控制量产

你现在还有几个点不属于“高端量产”：

```text
1. 目前还是开环，没有电流闭环
2. 控制频率目前是 1kHz，做高性能 FOC 通常偏低
3. 控制循环里不能 1ms 打印一次 ESP_LOGI
4. MCPWM 当前是边沿对齐 UP 模式，不是中心对齐
5. 还没有电流采样同步点
6. 还没有过流、欠压、堵转、温度保护闭环策略
7. 没有启动斜坡、对齐、开环切闭环流程
```

你的当前 MCPWM 是 20kHz PWM、10MHz 分辨率、500 tick 周期，并且控制周期是 1kHz。这个可以作为学习和低速开环验证，但后续要做更高性能，就要把控制周期提高，PWM 改中心对齐，并配合同步电流采样。

---

# 7. 测试参数建议

刚换这版 SVPWM，先这样：

```c
#define M0_TEST_RPM                100.0f
#define M0_TEST_UQ                 1.0f
#define MOTOR_DRV_VBUS             12.0f
```

稳定后再上：

```c
#define M0_TEST_RPM                300.0f
#define M0_TEST_UQ                 2.0f
```

因为这版会自动限制线性调制区，所以 `Uq` 给太大不会直接把 duty 打爆，但电机仍然可能因为开环角度、电压、负载不匹配产生啸叫或抖动。

---

你现在真正要替换的核心就两处：

```text
1. vik_foc.c：
   新增 vfoc_svpwm_calc_duty_ab()
   新增 vfoc_open_loop_svpwm_run()

2. m0_fd6287_pwm.c：
   把 vfoc_open_loop_spwm_run()
   换成 vfoc_open_loop_svpwm_run()
```

这版才是可以继续往“产品级 FOC”扩展的 SVPWM 写法。
