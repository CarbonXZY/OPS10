#ifndef QUATERNION_EKF_HPP
#define QUATERNION_EKF_HPP

#include "kalman_filter.hpp"
#include <array>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

template <size_t X_SIZE>
inline std::array<float, X_SIZE * X_SIZE> MakePInit()
{
    std::array<float, X_SIZE * X_SIZE> arr = {0};
    for (size_t i = 0; i < X_SIZE; i++)
        for (size_t j = 0; j < X_SIZE; j++)
            arr[i * X_SIZE + j] = (i == j) ? (i < 4 ? 100000.0f : 100.0f) : 0.1f;
    return arr;
}

template <size_t X_SIZE>
inline std::array<float, X_SIZE * X_SIZE> MakeAInit()
{
    std::array<float, X_SIZE * X_SIZE> arr = {0};
    for (size_t i = 0; i < X_SIZE; i++)
        arr[i * X_SIZE + i] = 1.0f;
    return arr;
}

/**
 * @brief 四元数EKF，融合陀螺仪、加速度计、磁力计数据
 *
 * 状态向量 (9维):
 *   [0-3]: 四元数 (q0, q1, q2, q3)
 *   [4-6]: 陀螺仪零偏 (bias_x, bias_y, bias_z)
 *
 * 观测向量:
 *   加速度计: 3维 - 重力方向
 *   磁力计:   3维 - 磁场方向
 */
template <size_t X_SIZE, size_t Z_SIZE>
class QuaternionEKF : public Class_KalmanFilter<X_SIZE, 0, Z_SIZE>
{
public:
    QuaternionEKF();

    /**
     * @brief 初始化四元数EKF
     * @param process_noise_q 四元数过程噪声 (建议: 10)
     * @param process_noise_bias 陀螺仪零偏过程噪声 (建议: 0.001)
     * @param accel_measure_noise 加速度计量测噪声 (建议: 1000000)
     * @param mag_measure_noise 磁力计量测噪声 (建议: 100000)
     * @param lambda 渐消因子 (0.95-1.0，建议: 0.9996)
     * @param acc_lpf_coef 加速度计低通滤波系数 (建议: 0)
     * @param mag_lpf_coef 磁力计低通滤波系数 (建议: 0)
     * @param use_magnetometer 是否使用磁力计
     */
    void Init(float process_noise_q = 10.0f,
              float process_noise_bias = 0.001f,
              float accel_measure_noise = 1000000.0f,
              float mag_measure_noise = 100000.0f,
              float lambda = 0.9996f,
              float acc_lpf_coef = 0.0f,
              float mag_lpf_coef = 0.0f,
              bool use_magnetometer = false);

    void Update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz,
                float dt);

    void UpdateAccelOnly(float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float dt);

    void Reset();

    // 获取结果
    inline std::array<float, 4> GetQuaternion() const
    {
        return {this->xhat_data[0], this->xhat_data[1], this->xhat_data[2], this->xhat_data[3]};
    }

    inline float GetRoll() const { return m_roll; }
    inline float GetPitch() const { return m_pitch; }
    inline float GetYaw() const { return m_yaw; }
    inline float GetYawTotal() const { return m_yaw_total; }
    inline std::array<float, 3> GetGyroBias() const
    {
        return {this->xhat_data[4], this->xhat_data[5], this->xhat_data[6]};
    }
    inline bool IsConverged() const { return m_converge_flag; }
    inline bool IsStable() const { return m_stable_flag; }

protected:
    void User_Func_Linearization_P_Fading() override;
    void User_Func_SetH() override;
    void User_Func_xhatUpdate() override;
    void User_Func_Observe() override;

    void UpdateEulerAngles();
    void ChiSquareTest();

    bool m_mag_valid; // 动态裁剪: 本周期磁力计数据是否有效

private:
    float m_Q1;      // 四元数过程噪声
    float m_Q2;      // 陀螺仪零偏过程噪声
    float m_R_accel; // 加速度计量测噪声
    float m_R_mag;   // 磁力计量测噪声
    float m_lambda;  // 渐消因子
    float m_acc_lpf_coef;
    float m_mag_lpf_coef;
    bool m_use_magnetometer;

    float m_dt;
    float m_gyro[3];
    float m_accel[3];
    float m_mag[3];
    float m_accel_filtered[3];
    float m_mag_filtered[3];
    float m_orientation_cosine[3];
    float m_adaptive_gain_scale;
    float m_chi_square;
    float m_chi_square_threshold;

    bool m_converge_flag;
    bool m_stable_flag;
    uint32_t m_error_count;
    uint32_t m_update_count;

    float m_roll;
    float m_pitch;
    float m_yaw;
    float m_yaw_total;
    float m_yaw_last;
    int16_t m_yaw_round_count;
};

// ==================== 构造函数 ====================
template <size_t X_SIZE, size_t Z_SIZE>
QuaternionEKF<X_SIZE, Z_SIZE>::QuaternionEKF()
    : m_chi_square_threshold(10.0f), m_converge_flag(false), m_stable_flag(false), m_error_count(0), m_update_count(0), m_yaw_round_count(0), m_yaw_last(0), m_yaw_total(0), m_mag_valid(true)
{
}

// ==================== 初始化 ====================
template <size_t X_SIZE, size_t Z_SIZE>
void QuaternionEKF<X_SIZE, Z_SIZE>::Init(float process_noise_q,
                                         float process_noise_bias,
                                         float accel_measure_noise,
                                         float mag_measure_noise,
                                         float lambda,
                                         float acc_lpf_coef,
                                         float mag_lpf_coef,
                                         bool use_magnetometer)
{
    m_Q1 = process_noise_q;
    m_Q2 = process_noise_bias;
    m_R_accel = accel_measure_noise;
    m_R_mag = mag_measure_noise;
    m_lambda = (lambda > 1.0f) ? 1.0f : lambda;
    m_acc_lpf_coef = acc_lpf_coef;
    m_mag_lpf_coef = mag_lpf_coef;
    m_use_magnetometer = use_magnetometer;
    m_mag_valid = true;

    // 设置初始状态（单位四元数）
    this->xhat_data.fill(0);
    this->xhat_data[0] = 1.0f;

    // 设置初始协方差矩阵
    this->P_data = MakePInit<X_SIZE>();

    // 状态转移矩阵初始为单位阵
    this->A_data = MakeAInit<X_SIZE>();

    // 观测噪声矩阵（对角线由参数决定）
    this->R_data.fill(0);
    for (size_t i = 0; i < 3; i++)
        this->R_data[i * Z_SIZE + i] = m_R_accel;
    if (m_use_magnetometer && Z_SIZE >= 6)
        for (size_t i = 3; i < 6; i++)
            this->R_data[i * Z_SIZE + i] = m_R_mag;

    // 启用动态调整
    this->UseAutoAdjustment = true;

    // 测量映射
    this->MeasurementMap.fill(0);
    this->MeasurementMap[0] = 1;
    this->MeasurementMap[1] = 2;
    this->MeasurementMap[2] = 3;
    if (m_use_magnetometer && Z_SIZE >= 6)
    {
        this->MeasurementMap[3] = 1;
        this->MeasurementMap[4] = 2;
        this->MeasurementMap[5] = 3;
    }
    this->MeasurementDegree.fill(1.0f);

    // 观测噪声对角线
    this->MatR_DiagonalElements.fill(0);
    for (size_t i = 0; i < 3; i++)
    {
        this->MatR_DiagonalElements[i] = m_R_accel;
    }
    if (m_use_magnetometer && Z_SIZE >= 6)
    {
        for (size_t i = 3; i < 6; i++)
        {
            this->MatR_DiagonalElements[i] = m_R_mag;
        }
    }

    // 跳过标准步骤
    this->SkipEq3 = TRUE;
    this->SkipEq4 = TRUE;

    m_converge_flag = false;
    m_error_count = 0;
    m_update_count = 0;

    m_accel_filtered[0] = m_accel_filtered[1] = m_accel_filtered[2] = 0;
    m_mag_filtered[0] = m_mag_filtered[1] = m_mag_filtered[2] = 0;
}

template <size_t X_SIZE, size_t Z_SIZE>
void QuaternionEKF<X_SIZE, Z_SIZE>::Reset()
{
    Class_KalmanFilter<X_SIZE, 0, Z_SIZE>::Reset();

    // 重置状态向量（单位四元数，零偏置）
    this->xhat_data.fill(0);
    this->xhat_data[0] = 1.0f; // 四元数 w=1, x=y=z=0

    // 重置先验状态
    this->xhatminus_data.fill(0);
    this->xhatminus_data[0] = 1.0f;

    // 重置协方差矩阵（与Init中的设置一致）
    this->P_data = MakePInit<X_SIZE>();

    // 重置先验协方差
    memcpy(this->Pminus_data.data(), this->P_data.data(), sizeof(float) * X_SIZE * X_SIZE);

    // 重置输入缓冲区
    this->z_input.fill(0);
    if (m_use_magnetometer)
    {
        this->MeasurementMap.fill(0);
        this->MeasurementMap[0] = 1;
        this->MeasurementMap[1] = 2;
        this->MeasurementMap[2] = 3;
        if constexpr (Z_SIZE >= 6)
        {
            this->MeasurementMap[3] = 1;
            this->MeasurementMap[4] = 2;
            this->MeasurementMap[5] = 3;
        }
        this->MeasurementDegree.fill(1.0f);

        for (size_t i = 0; i < 3 && i < Z_SIZE; i++)
            this->MatR_DiagonalElements[i] = m_R_accel;
        if (m_use_magnetometer && Z_SIZE >= 6)
        {
            for (size_t i = 3; i < 6; i++)
                this->MatR_DiagonalElements[i] = m_R_mag;
        }
    }

    // 重置欧拉角
    m_roll = 0.0f;
    m_pitch = 0.0f;
    m_yaw = 0.0f;
    m_yaw_total = 0.0f;
    m_yaw_last = 0.0f;
    m_yaw_round_count = 0;

    // 重置收敛和稳定标志
    m_converge_flag = false;
    m_stable_flag = false;
    m_error_count = 0;
    m_update_count = 0;
    m_mag_valid = true;

    // 重置自适应增益
    m_adaptive_gain_scale = 1.0f;
    m_chi_square = 0.0f;

    m_chi_square_threshold = 10.0f;
    // 重置滤波后的传感器数据
    m_accel_filtered[0] = m_accel_filtered[1] = m_accel_filtered[2] = 0;
    m_mag_filtered[0] = m_mag_filtered[1] = m_mag_filtered[2] = 0;

    // 重置方向余弦
    m_orientation_cosine[0] = m_orientation_cosine[1] = m_orientation_cosine[2] = 0;

    // 重置基类的跳过标志（保持与Init一致）
    this->SkipEq3 = TRUE;
    this->SkipEq4 = TRUE;

    // 可选：重置A矩阵为单位阵
    for (size_t i = 0; i < X_SIZE; i++)
    {
        for (size_t j = 0; j < X_SIZE; j++)
        {
            this->A_data[i * X_SIZE + j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    // 重置Q矩阵
    this->Q_data.fill(0);
    for (size_t i = 0; i < 4; i++)
        this->Q_data[i * X_SIZE + i] = m_Q1; // 注意：稍后在Update中会乘以dt
    for (size_t i = 4; i < 7 && i < X_SIZE; i++)
        this->Q_data[i * X_SIZE + i] = m_Q2;

    // 重置R矩阵
    this->R_data.fill(0);
    for (size_t i = 0; i < 3 && i < Z_SIZE; i++)
        this->R_data[i * Z_SIZE + i] = m_R_accel;
    if (m_use_magnetometer && Z_SIZE >= 6)
    {
        for (size_t i = 3; i < 6; i++)
            this->R_data[i * Z_SIZE + i] = m_R_mag;
    }
}

static inline float fastInvSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// ==================== 线性化和渐消 ====================
template <size_t X_SIZE, size_t Z_SIZE>
void QuaternionEKF<X_SIZE, Z_SIZE>::User_Func_Linearization_P_Fading()
{
    float q0 = this->xhatminus_data[0];
    float q1 = this->xhatminus_data[1];
    float q2 = this->xhatminus_data[2];
    float q3 = this->xhatminus_data[3];

    // 四元数归一化
    float inv_norm = fastInvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    this->xhatminus_data[0] = q0 * inv_norm;
    this->xhatminus_data[1] = q1 * inv_norm;
    this->xhatminus_data[2] = q2 * inv_norm;
    this->xhatminus_data[3] = q3 * inv_norm;

    // 更新F矩阵中陀螺仪偏置相关的部分 (4x3块矩阵)
    this->A_data[0 * X_SIZE + 4] = this->xhatminus_data[1] * m_dt / 2;
    this->A_data[0 * X_SIZE + 5] = this->xhatminus_data[2] * m_dt / 2;
    this->A_data[0 * X_SIZE + 6] = this->xhatminus_data[3] * m_dt / 2;

    this->A_data[1 * X_SIZE + 4] = -this->xhatminus_data[0] * m_dt / 2;
    this->A_data[1 * X_SIZE + 5] = this->xhatminus_data[3] * m_dt / 2;
    this->A_data[1 * X_SIZE + 6] = -this->xhatminus_data[2] * m_dt / 2;

    this->A_data[2 * X_SIZE + 4] = -this->xhatminus_data[3] * m_dt / 2;
    this->A_data[2 * X_SIZE + 5] = -this->xhatminus_data[0] * m_dt / 2;
    this->A_data[2 * X_SIZE + 6] = this->xhatminus_data[1] * m_dt / 2;

    this->A_data[3 * X_SIZE + 4] = this->xhatminus_data[2] * m_dt / 2;
    this->A_data[3 * X_SIZE + 5] = -this->xhatminus_data[1] * m_dt / 2;
    this->A_data[3 * X_SIZE + 6] = -this->xhatminus_data[0] * m_dt / 2;

    // 渐消滤波
    for (size_t i = 4; i < 7 && i < X_SIZE; i++)
    {
        this->P_data[i * X_SIZE + i] /= m_lambda;
        if (this->P_data[i * X_SIZE + i] > 10000.0f)
            this->P_data[i * X_SIZE + i] = 10000.0f;
    }

    // 过程噪声
    for (size_t i = 0; i < 4; i++)
        this->Q_data[i * X_SIZE + i] = m_Q1 * m_dt;
    for (size_t i = 4; i < 7 && i < X_SIZE; i++)
        this->Q_data[i * X_SIZE + i] = m_Q2 * m_dt;
}

// ==================== 观测矩阵 ====================
template <size_t X_SIZE, size_t Z_SIZE>
void QuaternionEKF<X_SIZE, Z_SIZE>::User_Func_SetH()
{
    float q0 = this->xhatminus_data[0];
    float q1 = this->xhatminus_data[1];
    float q2 = this->xhatminus_data[2];
    float q3 = this->xhatminus_data[3];

    float dq0 = 2.0f * q0;
    float dq1 = 2.0f * q1;
    float dq2 = 2.0f * q2;
    float dq3 = 2.0f * q3;

    this->H_data.fill(0);

    // 加速度计部分
    this->H_data[0 * X_SIZE + 0] = -dq2;
    this->H_data[0 * X_SIZE + 1] = dq3;
    this->H_data[0 * X_SIZE + 2] = -dq0;
    this->H_data[0 * X_SIZE + 3] = dq1;

    this->H_data[1 * X_SIZE + 0] = dq1;
    this->H_data[1 * X_SIZE + 1] = dq0;
    this->H_data[1 * X_SIZE + 2] = dq3;
    this->H_data[1 * X_SIZE + 3] = dq2;

    this->H_data[2 * X_SIZE + 0] = dq0;
    this->H_data[2 * X_SIZE + 1] = -dq1;
    this->H_data[2 * X_SIZE + 2] = -dq2;
    this->H_data[2 * X_SIZE + 3] = dq3;

    // 磁力计部分 (仅当磁力计数据有效时设置)
    if (m_use_magnetometer && m_mag_valid && Z_SIZE >= 6)
    {
        // 当地磁场参考值 [Hx, Hy, Hz]，需要根据实际位置标定
        // 这里使用常见值：北半球，Hx指向北，Hz向下
        float Hx = 0.5f; // 地磁场水平分量
        float Hz = 0.5f; // 地磁场垂直分量

        this->H_data[3 * X_SIZE + 0] = 2.0f * (q1 * Hx + q3 * Hz);
        this->H_data[3 * X_SIZE + 1] = 2.0f * (q0 * Hx + q2 * Hz);
        this->H_data[3 * X_SIZE + 2] = 2.0f * (q3 * Hx + q1 * Hz);
        this->H_data[3 * X_SIZE + 3] = 2.0f * (q2 * Hx + q0 * Hz);

        this->H_data[4 * X_SIZE + 0] = 2.0f * (q2 * Hx - q0 * Hz);
        this->H_data[4 * X_SIZE + 1] = 2.0f * (q3 * Hx - q1 * Hz);
        this->H_data[4 * X_SIZE + 2] = 2.0f * (q0 * Hx - q2 * Hz);
        this->H_data[4 * X_SIZE + 3] = 2.0f * (q1 * Hx - q3 * Hz);

        this->H_data[5 * X_SIZE + 0] = 2.0f * (q3 * Hx - q0 * Hz);
        this->H_data[5 * X_SIZE + 1] = 2.0f * (q2 * Hx - q1 * Hz);
        this->H_data[5 * X_SIZE + 2] = 2.0f * (q1 * Hx - q2 * Hz);
        this->H_data[5 * X_SIZE + 3] = 2.0f * (q0 * Hx - q3 * Hz);
    }
}

// ==================== 低通滤波 ====================
static void LowPassFilter3(float dt, float coef,
                           float input[3], float filtered[3],
                           uint32_t update_count)
{
    if (update_count == 0)
    {
        filtered[0] = input[0];
        filtered[1] = input[1];
        filtered[2] = input[2];
    }
    else
    {
        float alpha = dt / (dt + coef);
        float beta = coef / (dt + coef);
        filtered[0] = filtered[0] * beta + input[0] * alpha;
        filtered[1] = filtered[1] * beta + input[1] * alpha;
        filtered[2] = filtered[2] * beta + input[2] * alpha;
    }
}

// ==================== 欧拉角解算 ====================
template <size_t X_SIZE, size_t Z_SIZE>
void QuaternionEKF<X_SIZE, Z_SIZE>::UpdateEulerAngles()
{
    float q0 = this->xhat_data[0];
    float q1 = this->xhat_data[1];
    float q2 = this->xhat_data[2];
    float q3 = this->xhat_data[3];

    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    m_roll = atan2f(2.0f * (q2 * q3 + q0 * q1), q0q0 - q1q1 - q2q2 + q3q3) * 57.295779513f;
    m_pitch = asinf(-2.0f * (q1 * q3 - q0 * q2)) * 57.295779513f;
    m_yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), q0q0 + q1q1 - q2q2 - q3q3) * 57.295779513f;

    if (m_yaw - m_yaw_last > 180.0f)
        m_yaw_round_count--;
    else if (m_yaw - m_yaw_last < -180.0f)
        m_yaw_round_count++;
    m_yaw_total = 360.0f * m_yaw_round_count + m_yaw;
    m_yaw_last = m_yaw;
}

// ==================== 卡方检验 ====================
template <size_t X_SIZE, size_t Z_SIZE>
void QuaternionEKF<X_SIZE, Z_SIZE>::ChiSquareTest()
{
    float q0 = this->xhatminus_data[0];
    float q1 = this->xhatminus_data[1];
    float q2 = this->xhatminus_data[2];
    float q3 = this->xhatminus_data[3];

    float pred_gx = 2.0f * (q1 * q3 - q0 * q2);
    float pred_gy = 2.0f * (q0 * q1 + q2 * q3);
    float pred_gz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    m_orientation_cosine[0] = acosf(fabsf(pred_gx));
    m_orientation_cosine[1] = acosf(fabsf(pred_gy));
    m_orientation_cosine[2] = acosf(fabsf(pred_gz));

    float res_x = this->z_data[0] - pred_gx;
    float res_y = this->z_data[1] - pred_gy;
    float res_z = this->z_data[2] - pred_gz;
    m_chi_square = (res_x * res_x + res_y * res_y + res_z * res_z) / m_R_accel;

    if (m_chi_square < 0.5f * m_chi_square_threshold)
        m_converge_flag = true;

    if (m_chi_square > m_chi_square_threshold && m_converge_flag)
    {
        if (m_stable_flag)
            m_error_count++;
        else
            m_error_count = 0;
        if (m_error_count > 50)
            m_converge_flag = false;
    }
    else
    {
        if (m_chi_square > 0.1f * m_chi_square_threshold && m_converge_flag)
            m_adaptive_gain_scale = (m_chi_square_threshold - m_chi_square) / (0.9f * m_chi_square_threshold);
        else
            m_adaptive_gain_scale = 1.0f;
        m_error_count = 0;
    }
}

// ==================== 自定义状态更新 ====================
template <size_t X_SIZE, size_t Z_SIZE>
void QuaternionEKF<X_SIZE, Z_SIZE>::User_Func_xhatUpdate()
{
    float q0 = this->xhatminus_data[0];
    float q1 = this->xhatminus_data[1];
    float q2 = this->xhatminus_data[2];
    float q3 = this->xhatminus_data[3];

    // 预测重力方向
    float pred_gravity[3];
    pred_gravity[0] = 2.0f * (q1 * q3 - q0 * q2);
    pred_gravity[1] = 2.0f * (q0 * q1 + q2 * q3);
    pred_gravity[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 计算 inv(S) = inv(H·P'·HT + R)
    this->MatStatus = arm_mat_trans_f32(&this->H, &this->HT);
    this->MatStatus = arm_mat_mult_f32(&this->H, &this->Pminus, &this->temp_matrix);
    this->MatStatus = arm_mat_mult_f32(&this->temp_matrix, &this->HT, &this->temp_matrix1);
    this->MatStatus = arm_mat_add_f32(&this->temp_matrix1, &this->R, &this->S);
    this->MatStatus = arm_mat_inverse_f32(&this->S, &this->temp_matrix1); // temp_matrix1 = inv(S)

    // 残差
    float residual[6] = {0};
    for (int i = 0; i < 3; i++)
        residual[i] = this->z_data[i] - pred_gravity[i];

    // 磁力计残差 (仅当磁力计数据有效时计算)
    if (m_use_magnetometer && m_mag_valid && Z_SIZE >= 6)
    {
        // 预测磁场方向
        float Hx = 0.5f, Hz = 0.5f;
        float pred_mag_x = 2.0f * (q1 * q3 * Hx + q0 * q2 * Hx + q0 * q0 * Hz + q1 * q1 * Hz - 0.5f * Hz);
        float pred_mag_y = 2.0f * (q2 * q3 * Hx + q0 * q1 * Hx + q1 * q2 * Hz + q0 * q3 * Hz);
        float pred_mag_z = 2.0f * (q0 * q0 * Hx + q3 * q3 * Hx - 0.5f * Hx + q1 * q3 * Hz + q0 * q2 * Hz);

        residual[3] = this->z_data[3] - pred_mag_x;
        residual[4] = this->z_data[4] - pred_mag_y;
        residual[5] = this->z_data[5] - pred_mag_z;
    }

    // 计算方向余弦（用于偏置增益加权）
    for (uint8_t i = 0; i < 3; i++)
        m_orientation_cosine[i] = acosf(fabsf(pred_gravity[i]));

    // 卡方检验: rk' * inv(S) * rk
    const uint8_t nv = this->MeasurementValidNum > 0 ? this->MeasurementValidNum : static_cast<uint8_t>(Z_SIZE);
    float rk_data[6] = {0};
    for (uint8_t i = 0; i < nv; i++)
        rk_data[i] = residual[i];

    arm_matrix_instance_f32 rk, rkT_rk;
    float rkT_data[1];
    arm_mat_init_f32(&rk, nv, 1, rk_data);
    arm_mat_init_f32(&rkT_rk, 1, 1, rkT_data);

    arm_matrix_instance_f32 temp;
    float temp_data[6] = {0};
    arm_mat_init_f32(&temp, nv, 1, temp_data);

    // 当 nv < Z_SIZE 时，取 inv(S) 的左上角 nv×nv 子矩阵
    arm_matrix_instance_f32 S_sub;
    float S_sub_data[49]; // 最大 7×7 (X_SIZE=7, Z_SIZE=6, nv最大为6)
    if (nv < Z_SIZE)
    {
        arm_mat_init_f32(&S_sub, nv, nv, S_sub_data);
        for (uint8_t i = 0; i < nv; i++)
            for (uint8_t j = 0; j < nv; j++)
                S_sub_data[i * nv + j] = this->temp_matrix_data1[i * Z_SIZE + j];
    }

    arm_mat_mult_f32(nv < Z_SIZE ? &S_sub : &this->temp_matrix1, &rk, &temp); // temp = inv(S) * rk
    arm_mat_mult_f32(&rk, &temp, &rkT_rk);                                    // rkT_rk = rk' * inv(S) * rk
    m_chi_square = rkT_data[0];

    // 收敛和稳定判断
    if (m_chi_square < 0.5f * m_chi_square_threshold)
        m_converge_flag = true;

    if (m_chi_square > m_chi_square_threshold && m_converge_flag)
    {
        if (m_stable_flag)
            m_error_count++;
        else
            m_error_count = 0;
        if (m_error_count > 50)
        {
            m_converge_flag = false;
            this->SkipEq5 = FALSE; // step-5 is cov mat P updating
        }
        else
        {
            // 残差未通过卡方检验，仅预测
            memcpy(this->xhat_data.data(), this->xhatminus_data.data(), sizeof(float) * X_SIZE);
            memcpy(this->P_data.data(), this->Pminus_data.data(), sizeof(float) * X_SIZE * X_SIZE);
            this->SkipEq5 = TRUE; // part5 is P updating
            return;
        }
    }
    else
    {
        // scale adaptive, rk越小则增益越大,否则更相信预测值
        if (m_chi_square > 0.1f * m_chi_square_threshold && m_converge_flag)
        {
            m_adaptive_gain_scale = (m_chi_square_threshold - m_chi_square) / (0.9f * m_chi_square_threshold);
        }
        else
        {
            m_adaptive_gain_scale = 1.0f;
        }
        m_error_count = 0;
        this->SkipEq5 = FALSE;
    }

    // 计算卡尔曼增益 K = P'·HT·inv(S)
    // 当 nv < Z_SIZE 时,只取 HT 的前 nv 列和 inv(S) 的左上角 nv×nv
    if (nv < Z_SIZE)
    {
        // HT_sub: X_SIZE × nv, 从 temp_matrix 提取前 nv 列
        // temp_matrix1_sub: nv × nv, 从 temp_matrix1 提取左上角
        arm_matrix_instance_f32 HT_sub;
        arm_mat_init_f32(&HT_sub, X_SIZE, nv, &this->HT_data[0]);
        arm_matrix_instance_f32 S_sub_inv;
        arm_mat_init_f32(&S_sub_inv, nv, nv, S_sub_data);
        arm_matrix_instance_f32 K_temp;
        float K_temp_data[7 * 6];
        arm_mat_init_f32(&K_temp, X_SIZE, nv, K_temp_data);
        this->MatStatus = arm_mat_mult_f32(&this->Pminus, &HT_sub, &K_temp);
        this->MatStatus = arm_mat_mult_f32(&K_temp, &S_sub_inv, &this->K);
    }
    else
    {
        this->MatStatus = arm_mat_mult_f32(&this->Pminus, &this->HT, &this->temp_matrix);
        this->MatStatus = arm_mat_mult_f32(&this->temp_matrix, &this->temp_matrix1, &this->K);
    }

    // 自适应增益缩放
    for (size_t i = 0; i < X_SIZE; i++)
        for (uint8_t j = 0; j < nv; j++)
            this->K_data[i * Z_SIZE + j] *= m_adaptive_gain_scale;

    // 偏置增益按方向余弦加权
    for (size_t i = 4; i < 7 && i < X_SIZE; i++)
        for (uint8_t j = 0; j < nv; j++)
            this->K_data[i * Z_SIZE + j] *= m_orientation_cosine[i - 4] / 1.5707963f;

    // 计算修正量 temp = K * residual
    float correction_data[7] = {0};
    arm_matrix_instance_f32 correction;
    arm_mat_init_f32(&correction, X_SIZE, 1, correction_data);
    arm_matrix_instance_f32 rk_vec;
    arm_mat_init_f32(&rk_vec, nv, 1, rk_data);
    this->MatStatus = arm_mat_mult_f32(&this->K, &rk_vec, &correction);

    // 陀螺仪偏置修正限幅
    if (m_converge_flag)
    {
        for (size_t i = 4; i < 7 && i < X_SIZE; i++)
        {
            if (correction_data[i] > 1e-2f * m_dt)
                correction_data[i] = 1e-2f * m_dt;
            if (correction_data[i] < -1e-2f * m_dt)
                correction_data[i] = -1e-2f * m_dt;
        }
    }

    // 不修正q0数据
    correction_data[3] = 0;

    // 状态更新 xhat = xhatminus + correction
    for (size_t i = 0; i < X_SIZE; i++)
        this->xhat_data[i] = this->xhatminus_data[i] + correction_data[i];

    // 四元数归一化
    float inv_norm = fastInvSqrt(this->xhat_data[0] * this->xhat_data[0] +
                                 this->xhat_data[1] * this->xhat_data[1] +
                                 this->xhat_data[2] * this->xhat_data[2] +
                                 this->xhat_data[3] * this->xhat_data[3]);
    for (int i = 0; i < 4; i++)
        this->xhat_data[i] *= inv_norm;
}

template <size_t X_SIZE, size_t Z_SIZE>
void QuaternionEKF<X_SIZE, Z_SIZE>::User_Func_Observe()
{
}

// ==================== 主更新函数 ====================
template <size_t X_SIZE, size_t Z_SIZE>
void QuaternionEKF<X_SIZE, Z_SIZE>::Update(float gx, float gy, float gz,
                                           float ax, float ay, float az,
                                           float mx, float my, float mz,
                                           float dt)
{
    m_dt = dt;
    m_gyro[0] = gx - this->xhat_data[4];
    m_gyro[1] = gy - this->xhat_data[5];
    m_gyro[2] = gz - this->xhat_data[6]; // Z轴偏置现在被估计

    m_accel[0] = ax;
    m_accel[1] = ay;
    m_accel[2] = az;
    m_mag[0] = mx;
    m_mag[1] = my;
    m_mag[2] = mz;

    // 低通滤波
    LowPassFilter3(dt, m_acc_lpf_coef, m_accel, m_accel_filtered, m_update_count);
    if (m_use_magnetometer)
        LowPassFilter3(dt, m_mag_lpf_coef, m_mag, m_mag_filtered, m_update_count);

    // 归一化
    float acc_norm = fastInvSqrt(m_accel_filtered[0] * m_accel_filtered[0] +
                                 m_accel_filtered[1] * m_accel_filtered[1] +
                                 m_accel_filtered[2] * m_accel_filtered[2]);
    this->z_input[0] = m_accel_filtered[0] * acc_norm;
    this->z_input[1] = m_accel_filtered[1] * acc_norm;
    this->z_input[2] = m_accel_filtered[2] * acc_norm;

    if (m_use_magnetometer && Z_SIZE >= 6 && m_mag_valid)
    {
        float mag_norm = fastInvSqrt(m_mag_filtered[0] * m_mag_filtered[0] +
                                     m_mag_filtered[1] * m_mag_filtered[1] +
                                     m_mag_filtered[2] * m_mag_filtered[2]);
        this->z_input[3] = m_mag_filtered[0] * mag_norm;
        this->z_input[4] = m_mag_filtered[1] * mag_norm;
        this->z_input[5] = m_mag_filtered[2] * mag_norm;
    }

    // 更新状态转移矩阵（陀螺仪部分）
    float half_gx_dt = 0.5f * m_gyro[0] * dt;
    float half_gy_dt = 0.5f * m_gyro[1] * dt;
    float half_gz_dt = 0.5f * m_gyro[2] * dt;

    this->A_data.fill(0);
    for (size_t i = 0; i < X_SIZE; i++)
        this->A_data[i * X_SIZE + i] = 1.0f;

    this->A_data[0 * X_SIZE + 1] = -half_gx_dt;
    this->A_data[0 * X_SIZE + 2] = -half_gy_dt;
    this->A_data[0 * X_SIZE + 3] = -half_gz_dt;
    this->A_data[1 * X_SIZE + 0] = half_gx_dt;
    this->A_data[1 * X_SIZE + 2] = half_gz_dt;
    this->A_data[1 * X_SIZE + 3] = -half_gy_dt;
    this->A_data[2 * X_SIZE + 0] = half_gy_dt;
    this->A_data[2 * X_SIZE + 1] = -half_gz_dt;
    this->A_data[2 * X_SIZE + 3] = half_gx_dt;
    this->A_data[3 * X_SIZE + 0] = half_gz_dt;
    this->A_data[3 * X_SIZE + 1] = half_gy_dt;
    this->A_data[3 * X_SIZE + 2] = -half_gx_dt;

    // 判断稳定性
    float gyro_norm = fabsf(m_gyro[0]) + fabsf(m_gyro[1]) + fabsf(m_gyro[2]);
    float acc_norm_val = 1.0f / acc_norm;
    m_stable_flag = (gyro_norm < 0.3f && acc_norm_val > 9.3f && acc_norm_val < 10.3f);

    // 执行更新
    this->Class_KalmanFilter<X_SIZE, 0, Z_SIZE>::Update();

    UpdateEulerAngles();
    m_update_count++;
}

template <size_t X_SIZE, size_t Z_SIZE>
void QuaternionEKF<X_SIZE, Z_SIZE>::UpdateAccelOnly(float gx, float gy, float gz,
                                                    float ax, float ay, float az,
                                                    float dt)
{
    m_mag_valid = false; // 本周期仅使用加速度计
    Update(gx, gy, gz, ax, ay, az, 0, 0, 0, dt);
    m_mag_valid = true; // 恢复默认状态
}

#endif // QUATERNION_EKF_HPP