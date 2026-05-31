/**
 * @file    ahrs.c
 * @brief   Madgwick 9-axis AHRS 完整實作
 *
 * 原始演算法出自：
 *   S. Madgwick, "An efficient orientation filter for inertial and
 *   inertial/magnetic sensor arrays", 2010.
 *
 * 本實作重點：
 *   - 9 軸模式（陀螺儀 + 加速度計 + 磁力計）：提供絕對 Yaw（需磁力計）
 *   - 6 軸降級（mx=my=mz=0）：僅靠加速度計修正 Roll/Pitch，Yaw 依賴陀螺積分
 *   - 使用 Fast Inverse Square Root 降低 ARM 浮點運算開銷
 *   - 所有輸入向量在內部正規化，外部不需先做歸一化
 */

#include "ahrs.h"
#include <math.h>

/* ========== 濾波器狀態（模組內私有）========== */
static float beta;            /* 梯度下降步進增益 */
static float inv_sample_freq; /* 1.0 / sample_freq */

/* 四元數狀態，初始為單位旋轉 [1, 0, 0, 0] */
static float q0 = 1.0f;
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;

/* ========== 工具函式 ========== */

/**
 * Fast Inverse Square Root（Quake III Arena 演算法）
 * 在 ARM Cortex-M4 FPU 上速度快於 1/sqrtf()，誤差 < 0.2%
 */
static float inv_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long  i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y)); /* 第一次 Newton-Raphson 迭代 */
    y = y * (1.5f - (halfx * y * y)); /* 第二次迭代，精度更高 */
    return y;
}

/* ========== 公開介面 ========== */

void AHRS_Init(float _beta, float sample_freq)
{
    beta           = _beta;
    inv_sample_freq = 1.0f / sample_freq;
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
}

void AHRS_Update9(float gx, float gy, float gz,
                  float ax, float ay, float az,
                  float mx, float my, float mz)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;

    /* ---------- 1. 由陀螺儀估算四元數微分 ---------- */
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    /* ---------- 2. 感測器回饋修正 ---------- */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        /* 正規化加速度計 */
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        /* ---- 2a. 9 軸：有磁力計時計算完整修正（包含 Yaw）---- */
        if (!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {

            /* 正規化磁力計 */
            recipNorm = inv_sqrt(mx * mx + my * my + mz * mz);
            mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

            /* 輔助中間變數，避免重複乘法 */
            float _2q0mx  = 2.0f * q0 * mx;
            float _2q0my  = 2.0f * q0 * my;
            float _2q0mz  = 2.0f * q0 * mz;
            float _2q1mx  = 2.0f * q1 * mx;
            float _2q0    = 2.0f * q0;
            float _2q1    = 2.0f * q1;
            float _2q2    = 2.0f * q2;
            float _2q3    = 2.0f * q3;
            float _2q0q2  = 2.0f * q0 * q2;
            float _2q2q3  = 2.0f * q2 * q3;
            float q0q0    = q0 * q0;
            float q0q1    = q0 * q1;
            float q0q2    = q0 * q2;
            float q0q3    = q0 * q3;
            float q1q1    = q1 * q1;
            float q1q2    = q1 * q2;
            float q1q3    = q1 * q3;
            float q2q2    = q2 * q2;
            float q2q3    = q2 * q3;
            float q3q3    = q3 * q3;

            /* 地球磁場參考方向（投影到水平面的 North-Down 分量）*/
            float hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2
                     + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3
                     - mx * q2q2 - mx * q3q3;
            float hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1
                     + _2q1mx * q2 - my * q1q1 + my * q2q2
                     + _2q2 * mz * q3 - my * q3q3;
            float _2bx = sqrtf(hx * hx + hy * hy);
            float _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0
                       + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3
                       - mz * q2q2 + mz * q3q3;
            float _4bx = 2.0f * _2bx;
            float _4bz = 2.0f * _2bz;

            /* 梯度下降修正向量 (9 軸) */
            s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax)
               + _2q1 * (2.0f * q0q1 + _2q2q3 - ay)
               - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
               + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
               + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

            s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax)
               + _2q0 * (2.0f * q0q1 + _2q2q3 - ay)
               - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az)
               + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
               + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
               + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

            s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax)
               + _2q3 * (2.0f * q0q1 + _2q2q3 - ay)
               - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az)
               + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
               + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
               + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

            s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax)
               + _2q2 * (2.0f * q0q1 + _2q2q3 - ay)
               + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
               + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
               + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        } else {
            /* ---- 2b. 6 軸降級：無磁力計，僅修正 Roll/Pitch ---- */
            float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
            float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;
            float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
            float q0q0 = q0 * q0, q1q1 = q1 * q1;
            float q2q2 = q2 * q2, q3q3 = q3 * q3;

            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay
               - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay
               - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        }

        /* 正規化修正向量並套用 */
        recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm; s1 *= recipNorm;
        s2 *= recipNorm; s3 *= recipNorm;

        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    /* ---------- 3. 積分，更新四元數 ---------- */
    q0 += qDot1 * inv_sample_freq;
    q1 += qDot2 * inv_sample_freq;
    q2 += qDot3 * inv_sample_freq;
    q3 += qDot4 * inv_sample_freq;

    /* 正規化四元數，保持單位長度 */
    recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; q1 *= recipNorm;
    q2 *= recipNorm; q3 *= recipNorm;
}

void AHRS_GetQuaternion(float *out_q0, float *out_q1,
                        float *out_q2, float *out_q3)
{
    *out_q0 = q0;
    *out_q1 = q1;
    *out_q2 = q2;
    *out_q3 = q3;
}
