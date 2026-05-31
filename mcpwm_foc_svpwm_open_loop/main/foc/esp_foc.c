/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 修改说明：
 * 1. 原版使用 TI 风格 IQmath 定点数学库：_iq、_IQ()、_IQmpy()、_IQsin()、_IQcos()。
 * 2. 这里全部改成 float 浮点运算：float、普通乘法 *、sinf()、cosf()。
 * 3. 对新手更友好，公式更直观，方便用串口打印调试。
 */

#include "esp_foc.h"

/*
 * Clarke 变换：三相静止坐标系 U/V/W -> 两相静止坐标系 alpha/beta
 *
 * 为什么要做 Clarke 变换？
 * 电机本来是三相的，但是三相电流/电压相互之间有约束关系，
 * 对称三相里通常满足：u + v + w = 0。
 * 所以三相信息可以等效压缩成二维平面上的 alpha/beta。
 *
 * 等幅值 Clarke 公式：
 * alpha = 2/3 * u - 1/3 * v - 1/3 * w
 * beta  = sqrt(3)/3 * (v - w)
 *
 * 注意：
 * 这里的 v_uvw 不一定非得是“电压”，也可以是三相电流，
 * 具体看你调用这个函数时传进来的物理量是什么。
 */
void foc_clarke_transform(const foc_uvw_coord_t *v_uvw, foc_ab_coord_t *v_ab)
{
    const float k1 = 2.0f / 3.0f;
    const float k2 = 1.0f / 3.0f;
    const float k3 = (float)M_SQRT3 / 3.0f;

    /* alpha 轴：可以简单理解为和 U 相方向对齐的轴 */
    v_ab->alpha = v_uvw->u * k1 - (v_uvw->v + v_uvw->w) * k2;

    /* beta 轴：和 alpha 垂直的另一根轴 */
    v_ab->beta = (v_uvw->v - v_uvw->w) * k3;
}

/*
 * 反 Clarke 变换：两相静止坐标系 alpha/beta -> 三相静止坐标系 U/V/W
 *
 * 这个函数常用于 FOC 输出阶段：
 * 控制算法算出 alpha/beta 电压后，要重新变成 U/V/W 三相，
 * 然后才能交给 PWM/SVPWM 输出到三相桥。
 *
 * 公式：
 * u = alpha
 * v = (sqrt(3) * beta - alpha) / 2
 * w = -u - v
 */
void foc_inverse_clarke_transform(const foc_ab_coord_t *v_ab, foc_uvw_coord_t *v_uvw)
{
    v_uvw->u = v_ab->alpha;
    v_uvw->v = ((float)M_SQRT3 * v_ab->beta - v_ab->alpha) * 0.5f;
    v_uvw->w = -v_uvw->u - v_uvw->v;
}

/*
 * Park 变换：两相静止坐标系 alpha/beta -> 两相旋转坐标系 d/q
 *
 * theta_rad 是电角度，单位是弧度，不是角度。
 * 例如：
 * 角度 180° = 弧度 pi
 * 角度 360° = 弧度 2*pi
 *
 * 为什么要做 Park 变换？
 * alpha/beta 坐标系是固定不动的，电机转子在转，控制起来不直观。
 * Park 变换会把坐标系跟着转子一起旋转。
 * 这样交流量看起来就像直流量，更方便 PI 控制。
 *
 * 公式：
 * d = alpha * cos(theta) + beta * sin(theta)
 * q = beta  * cos(theta) - alpha * sin(theta)
 */
void foc_park_transform(float theta_rad, const foc_ab_coord_t *v_ab, foc_dq_coord_t *v_dq)
{
    const float sin_theta = sinf(theta_rad);
    const float cos_theta = cosf(theta_rad);

    v_dq->d = v_ab->alpha * cos_theta + v_ab->beta * sin_theta;
    v_dq->q = v_ab->beta * cos_theta - v_ab->alpha * sin_theta;
}

/*
 * 反 Park 变换：两相旋转坐标系 d/q -> 两相静止坐标系 alpha/beta
 *
 * FOC 输出阶段常用这个函数。
 * 比如你给定：
 * d 轴电压 Vd
 * q 轴电压 Vq
 * 再结合当前电角度 theta，就可以算出静止坐标系下的 alpha/beta 电压。
 *
 * 公式：
 * alpha = d * cos(theta) - q * sin(theta)
 * beta  = q * cos(theta) + d * sin(theta)
 */
void foc_inverse_park_transform(float theta_rad, const foc_dq_coord_t *v_dq, foc_ab_coord_t *v_ab)
{
    const float sin_theta = sinf(theta_rad);
    const float cos_theta = cosf(theta_rad);

    v_ab->alpha = v_dq->d * cos_theta - v_dq->q * sin_theta;
    v_ab->beta = v_dq->q * cos_theta + v_dq->d * sin_theta;
}

/*
 * SVPWM 占空比计算：alpha/beta -> U/V/W 三相输出量
 *
 * SVPWM 的核心思想：
 * 逆变器三相桥一共有 8 种开关状态，其中 6 个是有效矢量，2 个是零矢量。
 * alpha/beta 平面被这 6 个有效矢量分成 6 个扇区，也叫 sextant。
 * 当前目标电压矢量落在哪个扇区，就用相邻两个有效矢量 + 零矢量合成它。
 *
 * 这个函数分两步：
 * 1. 判断当前 alpha/beta 目标矢量在哪个扇区。
 * 2. 根据扇区计算 U/V/W 三相的输出量。
 *
 * 注意：
 * 这里为了保持和原 Espressif 示例一致，输出 out_uvw 不是最终 MCPWM compare 值，
 * 后面通常还要做缩放和平移，例如：
 * duty_u = out_uvw.u / 2 + PWM_PERIOD / 4;
 */
void foc_svpwm_duty_calculate(const foc_ab_coord_t *v_ab, foc_uvw_coord_t *out_uvw)
{
    int sextant;

    const float alpha = v_ab->alpha;
    const float beta = v_ab->beta;
    const float sqrt3_alpha = (float)M_SQRT3 * alpha;

    /*
     * 第一步：判断目标电压矢量所在的 60° 扇区。
     *
     * beta > 0：在 alpha/beta 平面的上半部分。
     * beta < 0：在 alpha/beta 平面的下半部分。
     * alpha > 0：在右半部分。
     * alpha < 0：在左半部分。
     *
     * 再通过 beta 和 sqrt(3)*alpha 的关系，进一步判断具体扇区。
     */
    if (beta > 0.0f) {
        if (alpha > 0.0f) {
            /* 第一象限：可能是扇区 1 或 2 */
            if (beta > sqrt3_alpha) {
                sextant = 2;    /* 扇区 2：v2-v3 */
            } else {
                sextant = 1;    /* 扇区 1：v1-v2 */
            }
        } else {
            /* 第二象限：可能是扇区 2 或 3 */
            if (-beta > sqrt3_alpha) {
                sextant = 3;    /* 扇区 3：v3-v4 */
            } else {
                sextant = 2;    /* 扇区 2：v2-v3 */
            }
        }
    } else {
        if (alpha > 0.0f) {
            /* 第四象限：可能是扇区 5 或 6 */
            if (-beta > sqrt3_alpha) {
                sextant = 5;    /* 扇区 5：v5-v6 */
            } else {
                sextant = 6;    /* 扇区 6：v6-v1 */
            }
        } else {
            /* 第三象限：可能是扇区 4 或 5 */
            if (beta > sqrt3_alpha) {
                sextant = 4;    /* 扇区 4：v4-v5 */
            } else {
                sextant = 5;    /* 扇区 5：v5-v6 */
            }
        }
    }

    /*
     * 第二步：根据扇区计算三个相位的输出量。
     *
     * t1~t6 可以理解为当前扇区里相关基础矢量的作用时间/作用比例。
     * 不同扇区使用的相邻基础矢量不同，所以 switch 里公式也不同。
     */
    switch (sextant) {
    case 1: {
        /* 扇区 1：使用基础矢量 v1-v2 合成目标矢量 */
        const float t1 = -sqrt3_alpha + beta;
        const float t2 = -2.0f * beta;

        out_uvw->u = 0.5f * (1.0f - t1 - t2);
        out_uvw->v = out_uvw->u + t1;
        out_uvw->w = out_uvw->v + t2;
    } break;

    case 2: {
        /* 扇区 2：使用基础矢量 v2-v3 合成目标矢量 */
        const float t2 = -sqrt3_alpha - beta;
        const float t3 =  sqrt3_alpha - beta;

        out_uvw->v = 0.5f * (1.0f - t2 - t3);
        out_uvw->u = out_uvw->v + t3;
        out_uvw->w = out_uvw->u + t2;
    } break;

    case 3: {
        /* 扇区 3：使用基础矢量 v3-v4 合成目标矢量 */
        const float t3 = -2.0f * beta;
        const float t4 = sqrt3_alpha + beta;

        out_uvw->v = 0.5f * (1.0f - t3 - t4);
        out_uvw->w = out_uvw->v + t3;
        out_uvw->u = out_uvw->w + t4;
    } break;

    case 4: {
        /* 扇区 4：使用基础矢量 v4-v5 合成目标矢量 */
        const float t4 = sqrt3_alpha - beta;
        const float t5 = 2.0f * beta;

        out_uvw->w = 0.5f * (1.0f - t4 - t5);
        out_uvw->v = out_uvw->w + t5;
        out_uvw->u = out_uvw->v + t4;
    } break;

    case 5: {
        /* 扇区 5：使用基础矢量 v5-v6 合成目标矢量 */
        const float t5 =  sqrt3_alpha + beta;
        const float t6 = -sqrt3_alpha + beta;

        out_uvw->w = 0.5f * (1.0f - t5 - t6);
        out_uvw->u = out_uvw->w + t5;
        out_uvw->v = out_uvw->u + t6;
    } break;

    case 6: {
        /* 扇区 6：使用基础矢量 v6-v1 合成目标矢量 */
        const float t6 = 2.0f * beta;
        const float t1 = -sqrt3_alpha - beta;

        out_uvw->u = 0.5f * (1.0f - t6 - t1);
        out_uvw->w = out_uvw->u + t1;
        out_uvw->v = out_uvw->w + t6;
    } break;

    default:
        /* 理论上不会走到这里。保险处理：全部输出 0。 */
        out_uvw->u = 0.0f;
        out_uvw->v = 0.0f;
        out_uvw->w = 0.0f;
        break;
    }
}
