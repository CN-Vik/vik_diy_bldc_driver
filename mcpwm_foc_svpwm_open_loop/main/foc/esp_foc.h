/*
 * ESP32 FOC float version
 *
 * 这个头文件用于替换原来依赖 IQmathLib 的 esp_foc.h。
 * 原来的代码使用 _iq、_IQ()、_IQmpy() 做定点小数运算；
 * 这里全部改成 float，适合新手学习和调试。
 */
#pragma once

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 有些编译环境不一定默认提供 M_PI / M_SQRT3，
 * 这里手动补一下，避免编译时报未定义。
 */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_SQRT3
#define M_SQRT3 1.73205080756887729353
#endif

/*
 * 三相坐标系：U/V/W
 * 可以理解为电机三根相线上的电压或电流。
 */
typedef struct {
    float u;
    float v;
    float w;
} foc_uvw_coord_t;

/*
 * 两相静止坐标系：alpha/beta
 * Clarke 变换会把三相 U/V/W 转换成 alpha/beta。
 */
typedef struct {
    float alpha;
    float beta;
} foc_ab_coord_t;

/*
 * 两相旋转坐标系：d/q
 * Park 变换会把 alpha/beta 转换成 d/q。
 * d 轴通常理解为磁链方向，q 轴通常理解为转矩方向。
 */
typedef struct {
    float d;
    float q;
} foc_dq_coord_t;



/**
 * @brief Clarke 变换：把三相静止坐标系 U/V/W 转换成两相静止坐标系 alpha/beta
 *
 * 通俗理解：
 * 三相电机有 U、V、W 三根相线，但是 FOC 计算时不太方便直接用三相量。
 * Clarke 变换就是把三相量压缩成二维平面上的 alpha、beta 两个量。
 *
 * @param[in]  v_uvw  输入：三相 U/V/W 坐标数据
 * @param[out] v_ab   输出：两相 alpha/beta 坐标数据
 */
void foc_clarke_transform(const foc_uvw_coord_t *v_uvw, foc_ab_coord_t *v_ab);

/**
 * @brief 反 Clarke 变换：把两相静止坐标系 alpha/beta 转换回三相静止坐标系 U/V/W
 *
 * 通俗理解：
 * FOC 算法内部通常先算出 alpha、beta 电压，
 * 但是最终要控制三相逆变器，所以还需要还原成 U、V、W 三相输出。
 *
 * @param[in]  v_ab   输入：两相 alpha/beta 坐标数据
 * @param[out] v_uvw  输出：三相 U/V/W 坐标数据
 */
void foc_inverse_clarke_transform(const foc_ab_coord_t *v_ab, foc_uvw_coord_t *v_uvw);

/**
 * @brief Park 变换：把两相静止坐标系 alpha/beta 转换成旋转坐标系 d/q
 *
 * 通俗理解：
 * alpha/beta 坐标系是固定不动的，而电机转子是旋转的。
 * Park 变换就是站在“跟着转子一起旋转”的视角看电流/电压。
 *
 * d 轴：通常理解为磁场方向
 * q 轴：通常理解为产生转矩的方向
 *
 * @param[in]  theta_rad  电角度，单位是弧度 rad，用来表示当前转子磁场角度
 * @param[in]  v_ab       输入：两相 alpha/beta 坐标数据
 * @param[out] v_dq       输出：旋转坐标系 d/q 数据
 */
void foc_park_transform(float theta_rad, const foc_ab_coord_t *v_ab, foc_dq_coord_t *v_dq);

/**
 * @brief 反 Park 变换：把旋转坐标系 d/q 转换回两相静止坐标系 alpha/beta
 *
 * 通俗理解：
 * 在 FOC 控制里，我们通常希望直接控制 d 轴和 q 轴。
 * 但是逆变器最终不能直接输出 d/q，需要先转换回 alpha/beta，
 * 后面再通过 SVPWM 或 SPWM 生成三相 PWM。
 *
 * @param[in]  theta_rad  电角度，单位是弧度 rad，用来表示当前转子磁场角度
 * @param[in]  v_dq       输入：旋转坐标系 d/q 数据
 * @param[out] v_ab       输出：两相 alpha/beta 坐标数据
 */
void foc_inverse_park_transform(float theta_rad, const foc_dq_coord_t *v_dq, foc_ab_coord_t *v_ab);

/**
 * @brief SVPWM 占空比计算：根据 alpha/beta 电压计算三相 U/V/W PWM 占空比
 *
 * 通俗理解：
 * FOC 算出来的是一个二维电压矢量，也就是 alpha/beta。
 * 但是三相桥最终要输出 U、V、W 三路 PWM。
 * SVPWM 的作用就是把这个电压矢量转换成三相 PWM 占空比。
 *
 * 这里的 7 段 SVPWM 指的是：
 * 一个 PWM 周期内，会按照一定顺序插入有效矢量和零矢量，
 * 让三相桥输出的平均电压等效成我们想要的电压矢量。
 *
 * @param[in]  v_ab     输入：两相 alpha/beta 坐标数据
 * @param[out] out_uvw  输出：计算出来的三相 U/V/W PWM 占空比数据
 */
void foc_svpwm_duty_calculate(const foc_ab_coord_t *v_ab, foc_uvw_coord_t *out_uvw);



#ifdef __cplusplus
}
#endif
