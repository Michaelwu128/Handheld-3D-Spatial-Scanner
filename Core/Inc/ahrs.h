/**
 * @file    ahrs.h
 * @brief   Madgwick 9-axis AHRS (Attitude & Heading Reference System)
 *
 * 演算法：Sebastian Madgwick 2010 gradient-descent fusion
 * - 輸入：陀螺儀 (rad/s)、加速度計 (g)、磁力計 (任意單位，內部正規化)
 * - 輸出：四元數 q = [q0, q1, q2, q3]，代表 body-frame → world-frame 旋轉
 * - 磁力計輸入全為 0 時自動降級為 6 軸模式（無 Yaw 修正，但不崩潰）
 *
 * 參數建議：
 *   beta = 0.033f  → 動態響應快，適合快速掃描
 *   beta = 0.10f   → 收斂穩定，適合慢速精確掃描（預設）
 *   sample_freq    → 應與 IMU_Task 的呼叫頻率一致（例如 50.0f Hz）
 */

#ifndef AHRS_H_
#define AHRS_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  初始化 AHRS 濾波器，重置四元數為單位旋轉
 * @param  beta         梯度下降步進增益，建議 0.033~0.10
 * @param  sample_freq  感測器更新頻率 (Hz)
 */
void AHRS_Init(float beta, float sample_freq);

/**
 * @brief  9 軸 Madgwick 更新（若 mx=my=mz=0 自動降為 6 軸）
 * @param  gx,gy,gz     陀螺儀角速度 (rad/s)
 * @param  ax,ay,az     加速度計 (g)，內部會正規化，無需事先歸一化
 * @param  mx,my,mz     磁力計 (任意單位)，內部會正規化；全為 0 則略過磁力修正
 */
void AHRS_Update9(float gx, float gy, float gz,
                  float ax, float ay, float az,
                  float mx, float my, float mz);

/**
 * @brief  取得當前四元數估計值
 * @param  out_q0~q3    輸出指標（q0 為純量部分）
 */
void AHRS_GetQuaternion(float *out_q0, float *out_q1,
                        float *out_q2, float *out_q3);

#ifdef __cplusplus
}
#endif

#endif /* AHRS_H_ */
