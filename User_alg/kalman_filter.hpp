#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "arm_math.h"
#include <array>
#include <cstddef>
#include <cstring>
#include <algorithm>

template <size_t X_SIZE, size_t U_SIZE, size_t Z_SIZE>
class Class_KalmanFilter
{
public:
    // 构造函数 - 自动初始化矩阵实例
    Class_KalmanFilter() { Init(); }

		void Init();
    // 算法函数
    void xhatMinusUpdate(); // 先验状态估计更新
    void PminusUpdate();    // 先验估计误差协方差更新
    void SetKalmanGain();   // 卡尔曼增益计算
    void xhatUpdate();      // 状态估计更新
    void PUpdate();         // 估计误差协方差更新
    void Update();          // 观测更新

    // 重置滤波器
    void Reset();

    // 配合用户函数钩子的跳过标志位
    uint8_t SkipEq1 = 0, SkipEq2 = 0, SkipEq3 = 0, SkipEq4 = 0, SkipEq5 = 0;

    // 用户输入接口
    std::array<float, (Z_SIZE > 0) ? Z_SIZE : 1> z_input = {0}; // 观测值输入接口，用户需要在调用Update()前填充这个数组
    std::array<float, (U_SIZE > 0) ? U_SIZE : 1> u_input = {0}; // 控制输入接口，用户需要在调用Update()前填充这个数组
    std::array<float, X_SIZE> StateMinVariance = {0};

    // 配置参数（使用前必须设置）
    std::array<float, X_SIZE * X_SIZE> A_data = {0};  // 状态转移矩阵
    std::array<float, X_SIZE * X_SIZE> Q_data = {0};  // 过程噪声协方差
    std::array<float, X_SIZE> xhat_data = {0};        // 初始状态估计
    std::array<float, X_SIZE * X_SIZE> P_data = {0};  // 初始估计误差协方差（对角线设为较大值如100）

    // 动态量测裁剪配置（UseAutoAdjustment = true 时需要设置）
    std::array<uint8_t, Z_SIZE> MeasurementMap = {0};      // 量测映射表，1-based状态索引，0表示无效
    std::array<float, Z_SIZE> MeasurementDegree = {0};     // 量测重要程度，范围0-1
    std::array<float, Z_SIZE> MatR_DiagonalElements = {0}; // 观测噪声协方差对角线元素

    // 固定模式配置（UseAutoAdjustment = false 时需要设置）
    std::array<float, Z_SIZE * X_SIZE> H_data = {0}; // 观测矩阵
    std::array<float, Z_SIZE * Z_SIZE> R_data = {0}; // 观测噪声协方差

    // 功能开关
    bool UseAutoAdjustment = false; // 是否启用自动调整

    // 用户函数钩子（派生类可重写）
    virtual void User_Func_Observe() {};                // 用户实现观测更新逻辑
    virtual void User_Func_Linearization_P_Fading() {}; // 用户实现线性化和衰减逻辑
    virtual void User_Func_SetH() {};                   // 用户实现观测矩阵设置逻辑
    virtual void User_Func_xhatUpdate() {};             // 用户实现状态估计更新逻辑
    virtual void User_Func4() {};                       // 预留接口4
    virtual void User_Func5() {};                       // 预留接口5
    virtual void User_Func6() {};                       // 预留接口6

    // 获取滤波结果
    inline float* GetState() { return xhat_data.data(); }
    inline float GetState(size_t idx) { return (idx < X_SIZE) ? xhat_data[idx] : 0.0f; }

protected:
    void H_K_R_Adjustment(); // 动态量测裁剪算法实现
    void ApplyPClipping();   // 防止过度收敛

    // 矩阵存储
    std::array<float, X_SIZE> xhatminus_data = {0};
    std::array<float, (U_SIZE > 0) ? U_SIZE : 1> u_data = {0};
    std::array<float, (Z_SIZE > 0) ? Z_SIZE : 1> z_data = {0};
    std::array<float, X_SIZE * X_SIZE> Pminus_data = {0};
    std::array<float, X_SIZE * X_SIZE> AT_data = {0};
    std::array<float, (X_SIZE * U_SIZE > 0 ? X_SIZE * U_SIZE : 1)> B_data = {0};
    std::array<float, Z_SIZE * X_SIZE> HT_data = {0};
    std::array<float, X_SIZE * Z_SIZE> K_data = {0};

    // 中间运算变量（使用最大可能维度）
    static constexpr size_t MAX_DIM = (X_SIZE > Z_SIZE) ? X_SIZE : Z_SIZE;
    std::array<float, (Z_SIZE * Z_SIZE > 0) ? Z_SIZE * Z_SIZE : 1> S_data = {0};
    std::array<float, MAX_DIM * MAX_DIM> temp_matrix_data = {0};
    std::array<float, MAX_DIM * MAX_DIM> temp_matrix_data1 = {0};
    std::array<float, X_SIZE> temp_vector_data = {0};
    std::array<float, Z_SIZE> temp_vector_data1 = {0};

    // ARM CMSIS-DSP库矩阵结构体
    arm_matrix_instance_f32 xhat;
    arm_matrix_instance_f32 xhatminus;
    arm_matrix_instance_f32 u;
    arm_matrix_instance_f32 z;
    arm_matrix_instance_f32 P;
    arm_matrix_instance_f32 Pminus;
    arm_matrix_instance_f32 A;
    arm_matrix_instance_f32 AT;
    arm_matrix_instance_f32 B;
    arm_matrix_instance_f32 H;
    arm_matrix_instance_f32 HT;
    arm_matrix_instance_f32 Q;
    arm_matrix_instance_f32 R;
    arm_matrix_instance_f32 K;
    arm_matrix_instance_f32 S;
    arm_matrix_instance_f32 temp_matrix;
    arm_matrix_instance_f32 temp_matrix1;
    arm_matrix_instance_f32 temp_vector;
    arm_matrix_instance_f32 temp_vector1;

    // 滤波器状态变量
    int8_t MatStatus = 0;
    bool Initialized = false;
    uint8_t MeasurementValidNum = 0;
};

// ==================== 初始化 ====================
template <size_t X_SIZE, size_t U_SIZE, size_t Z_SIZE>
void Class_KalmanFilter<X_SIZE, U_SIZE, Z_SIZE>::Init()
{
    // 初始化ARM CMSIS-DSP库矩阵结构体
    arm_mat_init_f32(&xhat, X_SIZE, 1, xhat_data.data());
    arm_mat_init_f32(&xhatminus, X_SIZE, 1, xhatminus_data.data());
    
    if (U_SIZE > 0)
    {
        arm_mat_init_f32(&u, U_SIZE, 1, u_data.data());
        arm_mat_init_f32(&B, X_SIZE, U_SIZE, B_data.data());
    }
    
    arm_mat_init_f32(&z, Z_SIZE, 1, z_data.data());
    arm_mat_init_f32(&P, X_SIZE, X_SIZE, P_data.data());
    arm_mat_init_f32(&Pminus, X_SIZE, X_SIZE, Pminus_data.data());
    arm_mat_init_f32(&A, X_SIZE, X_SIZE, A_data.data());
    arm_mat_init_f32(&AT, X_SIZE, X_SIZE, AT_data.data());
    arm_mat_init_f32(&H, Z_SIZE, X_SIZE, H_data.data());
    arm_mat_init_f32(&HT, X_SIZE, Z_SIZE, HT_data.data());
    arm_mat_init_f32(&Q, X_SIZE, X_SIZE, Q_data.data());
    arm_mat_init_f32(&R, Z_SIZE, Z_SIZE, R_data.data());
    arm_mat_init_f32(&K, X_SIZE, Z_SIZE, K_data.data());
    
    arm_mat_init_f32(&S, Z_SIZE, Z_SIZE, S_data.data());
    arm_mat_init_f32(&temp_matrix, MAX_DIM, MAX_DIM, temp_matrix_data.data());
    arm_mat_init_f32(&temp_matrix1, MAX_DIM, MAX_DIM, temp_matrix_data1.data());
    arm_mat_init_f32(&temp_vector, X_SIZE, 1, temp_vector_data.data());
    arm_mat_init_f32(&temp_vector1, Z_SIZE, 1, temp_vector_data1.data());

    Initialized = true;
}

// ==================== 重置滤波器 ====================
template <size_t X_SIZE, size_t U_SIZE, size_t Z_SIZE>
void Class_KalmanFilter<X_SIZE, U_SIZE, Z_SIZE>::Reset()
{
    // 重置状态
    xhat_data.fill(0);
    xhatminus_data.fill(0);
    z_input.fill(0);
    
    if (U_SIZE > 0)
    {
        u_input.fill(0);
    }
    
    // 重置协方差（保持P矩阵不变，只重置状态）
    // 如果需要完全重置，用户应在外部重新设置 P_data
    
    MeasurementValidNum = 0;
    MatStatus = 0;
}

// ==================== 方程1: 先验状态估计 ====================
template <size_t X_SIZE, size_t U_SIZE, size_t Z_SIZE>
void Class_KalmanFilter<X_SIZE, U_SIZE, Z_SIZE>::xhatMinusUpdate()
{
    if (SkipEq1) return;

    if (U_SIZE > 0)
    {
        MatStatus = arm_mat_mult_f32(&A, &xhat, &temp_vector);      // temp = A * xhat
        MatStatus = arm_mat_mult_f32(&B, &u, &temp_vector1);        // temp1 = B * u
        MatStatus = arm_mat_add_f32(&temp_vector, &temp_vector1, &xhatminus);
    }
    else
    {
        MatStatus = arm_mat_mult_f32(&A, &xhat, &xhatminus);
    }
}

// ==================== 方程2: 先验协方差预测 ====================
template <size_t X_SIZE, size_t U_SIZE, size_t Z_SIZE>
void Class_KalmanFilter<X_SIZE, U_SIZE, Z_SIZE>::PminusUpdate()
{
    if (SkipEq2) return;

    MatStatus = arm_mat_trans_f32(&A, &AT);
    
    // Pminus = A * P * AT + Q
    MatStatus = arm_mat_mult_f32(&A, &P, &temp_matrix);     // temp = A * P
    MatStatus = arm_mat_mult_f32(&temp_matrix, &AT, &Pminus); // Pminus = A * P * AT
    MatStatus = arm_mat_add_f32(&Pminus, &Q, &Pminus);     // Pminus = Pminus + Q
}

// ==================== 方程3: 卡尔曼增益 ====================
template <size_t X_SIZE, size_t U_SIZE, size_t Z_SIZE>
void Class_KalmanFilter<X_SIZE, U_SIZE, Z_SIZE>::SetKalmanGain()
{
    if (SkipEq3) return;

    MatStatus = arm_mat_trans_f32(&H, &HT);
    
    // S = H * Pminus * HT + R
    MatStatus = arm_mat_mult_f32(&H, &Pminus, &temp_matrix);   // temp = H * Pminus
    MatStatus = arm_mat_mult_f32(&temp_matrix, &HT, &temp_matrix1); // temp1 = H * Pminus * HT
    MatStatus = arm_mat_add_f32(&temp_matrix1, &R, &S);        // S = temp1 + R
    
    MatStatus = arm_mat_inverse_f32(&S, &temp_matrix1);        // temp1 = inv(S)
    
    // K = Pminus * HT * inv(S)
    MatStatus = arm_mat_mult_f32(&Pminus, &HT, &temp_matrix);  // temp = Pminus * HT
    MatStatus = arm_mat_mult_f32(&temp_matrix, &temp_matrix1, &K); // K = temp * inv(S)
}

// ==================== 方程4: 后验状态更新 ====================
template <size_t X_SIZE, size_t U_SIZE, size_t Z_SIZE>
void Class_KalmanFilter<X_SIZE, U_SIZE, Z_SIZE>::xhatUpdate()
{
    if (SkipEq4) return;

    // xhat = xhatminus + K * (z - H * xhatminus)
    MatStatus = arm_mat_mult_f32(&H, &xhatminus, &temp_vector);   // temp = H * xhatminus
    MatStatus = arm_mat_sub_f32(&z, &temp_vector, &temp_vector1); // temp1 = z - H * xhatminus
    MatStatus = arm_mat_mult_f32(&K, &temp_vector1, &temp_vector); // temp = K * (z - H*xhatminus)
    MatStatus = arm_mat_add_f32(&xhatminus, &temp_vector, &xhat); // xhat = xhatminus + temp
}

// ==================== 方程5: 后验协方差更新 ====================
template <size_t X_SIZE, size_t U_SIZE, size_t Z_SIZE>
void Class_KalmanFilter<X_SIZE, U_SIZE, Z_SIZE>::PUpdate()
{
    if (SkipEq5) return;

    // P = Pminus - K * H * Pminus (数值更稳定)
    MatStatus = arm_mat_mult_f32(&K, &H, &temp_matrix);      // temp = K * H
    MatStatus = arm_mat_mult_f32(&temp_matrix, &Pminus, &temp_matrix1); // temp1 = K * H * Pminus
    MatStatus = arm_mat_sub_f32(&Pminus, &temp_matrix1, &P); // P = Pminus - K*H*Pminus
}

// ==================== 动态量测裁剪 ====================
template <size_t X_SIZE, size_t U_SIZE, size_t Z_SIZE>
void Class_KalmanFilter<X_SIZE, U_SIZE, Z_SIZE>::H_K_R_Adjustment()
{
    MeasurementValidNum = 0;

    // 复制用户输入
    memcpy(z_data.data(), z_input.data(), sizeof(z_input));
    z_input.fill(0);

    // 清零 H 和 R（每次全量重算）
    if constexpr (Z_SIZE > 0 && X_SIZE > 0) {
        H_data.fill(0);
        R_data.fill(0);
        for (size_t i = 0; i < Z_SIZE; i++)
            R_data[i * Z_SIZE + i] = 1.0f; // 默认对角线归1
    }

    // 逐行处理：有效行设置H/R，无效行保持H=0,R=1
    for (uint8_t i = 0; i < Z_SIZE; i++)
    {
        if (z_data[i] != 0.0f)
        {
            uint8_t state_idx = MeasurementMap[i] - 1;
            if (state_idx < X_SIZE)
            {
                H_data[i * X_SIZE + state_idx] = MeasurementDegree[i];
            }
            R_data[i * Z_SIZE + i] = MatR_DiagonalElements[i];
            MeasurementValidNum++;
        }
        // else: H行已清零, R对角线已归1, 该行完全不起作用
    }
}

// ==================== 防止过度收敛 ====================
template <size_t X_SIZE, size_t U_SIZE, size_t Z_SIZE>
void Class_KalmanFilter<X_SIZE, U_SIZE, Z_SIZE>::ApplyPClipping()
{
    bool has_limit = false;
    for (size_t i = 0; i < X_SIZE; i++)
    {
        if (StateMinVariance[i] > 0.0f)
        {
            has_limit = true;
            break;
        }
    }

    if (!has_limit)
    {
        const float P_min = 1e-6f;
        for (size_t i = 0; i < X_SIZE; i++)
        {
            if (P_data[i * X_SIZE + i] < P_min)
            {
                P_data[i * X_SIZE + i] = P_min;
            }
        }
    }
    else
    {
        for (size_t i = 0; i < X_SIZE; i++)
        {
            float min_var = StateMinVariance[i];
            if (min_var > 0.0f && P_data[i * X_SIZE + i] < min_var)
            {
                P_data[i * X_SIZE + i] = min_var;
            }
        }
    }
}

// ==================== 主更新函数 ====================
template <size_t X_SIZE, size_t U_SIZE, size_t Z_SIZE>
void Class_KalmanFilter<X_SIZE, U_SIZE, Z_SIZE>::Update()
{
    if (!Initialized) return;

    // 1. 消费输入并调整
    if (UseAutoAdjustment)
    {
        H_K_R_Adjustment();
    }
    else
    {
        memcpy(z_data.data(), z_input.data(), sizeof(z_input));
        z_input.fill(0);
        MeasurementValidNum = Z_SIZE;
    }

    // 2. 控制输入处理
    if (U_SIZE > 0)
    {
        memcpy(u_data.data(), u_input.data(), sizeof(u_input));
        u_input.fill(0);
    }

    // 3. 状态预测
    xhatMinusUpdate();
    User_Func_Observe();
    User_Func_Linearization_P_Fading();
    
    // 4. 协方差预测
    PminusUpdate();
    User_Func_SetH();

    // 5. 量测更新
    if (MeasurementValidNum > 0 || !UseAutoAdjustment)
    {
        SetKalmanGain();
        User_Func_xhatUpdate();
        xhatUpdate();
        User_Func4();
        PUpdate();
    }
    else
    {
        // 无有效测量，仅使用预测结果
        memcpy(xhat_data.data(), xhatminus_data.data(), sizeof(float) * X_SIZE);
        memcpy(P_data.data(), Pminus_data.data(), sizeof(float) * X_SIZE * X_SIZE);
    }

    User_Func5();
    ApplyPClipping();
    User_Func6();
}

#endif // KALMAN_FILTER_H