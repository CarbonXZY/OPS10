#ifndef ICM42688_REGISTERS_H_
#define ICM42688_REGISTERS_H_

#include <stdint.h>

// 所有用户Bank均可访问
#define REG_BANK_SEL             0x76

// ================= User Bank 0 =================
#define UB0_REG_DEVICE_CONFIG    0x11      // 设备配置寄存器
// break
#define UB0_REG_DRIVE_CONFIG     0x13      // 引脚驱动配置
#define UB0_REG_INT_CONFIG       0x14      // 中断配置
// break
#define UB0_REG_FIFO_CONFIG      0x16      // FIFO配置
// break
#define UB0_REG_TEMP_DATA1       0x1D      // 温度数据高字节
#define UB0_REG_TEMP_DATA0       0x1E      // 温度数据低字节
#define UB0_REG_ACCEL_DATA_X1    0x1F      // 加速度计X轴数据高字节
#define UB0_REG_ACCEL_DATA_X0    0x20      // 加速度计X轴数据低字节
#define UB0_REG_ACCEL_DATA_Y1    0x21      // 加速度计Y轴数据高字节
#define UB0_REG_ACCEL_DATA_Y0    0x22      // 加速度计Y轴数据低字节
#define UB0_REG_ACCEL_DATA_Z1    0x23      // 加速度计Z轴数据高字节
#define UB0_REG_ACCEL_DATA_Z0    0x24      // 加速度计Z轴数据低字节
#define UB0_REG_GYRO_DATA_X1     0x25      // 陀螺仪X轴数据高字节
#define UB0_REG_GYRO_DATA_X0     0x26      // 陀螺仪X轴数据低字节
#define UB0_REG_GYRO_DATA_Y1     0x27      // 陀螺仪Y轴数据高字节
#define UB0_REG_GYRO_DATA_Y0     0x28      // 陀螺仪Y轴数据低字节
#define UB0_REG_GYRO_DATA_Z1     0x29      // 陀螺仪Z轴数据高字节
#define UB0_REG_GYRO_DATA_Z0     0x2A      // 陀螺仪Z轴数据低字节
#define UB0_REG_TMST_FSYNCH      0x2B      // 时间戳/FSYNC高字节
#define UB0_REG_TMST_FSYNCL      0x2C      // 时间戳/FSYNC低字节
#define UB0_REG_INT_STATUS       0x2D      // 中断状态
#define UB0_REG_FIFO_COUNTH      0x2E      // FIFO数据量高字节
#define UB0_REG_FIFO_COUNTL      0x2F      // FIFO数据量低字节
#define UB0_REG_FIFO_DATA        0x30      // FIFO数据
#define UB0_REG_APEX_DATA0       0x31      // APEX数据0
#define UB0_REG_APEX_DATA1       0x32      // APEX数据1
#define UB0_REG_APEX_DATA2       0x33      // APEX数据2
#define UB0_REG_APEX_DATA3       0x34      // APEX数据3
#define UB0_REG_APEX_DATA4       0x35      // APEX数据4
#define UB0_REG_APEX_DATA5       0x36      // APEX数据5
#define UB0_REG_INT_STATUS2      0x37      // 中断状态2
#define UB0_REG_INT_STATUS3      0x38      // 中断状态3
// break
#define UB0_REG_SIGNAL_PATH_RESET    0x4B  // 信号路径复位
#define UB0_REG_INTF_CONFIG0         0x4C  // 接口配置0
#define UB0_REG_INTF_CONFIG1         0x4D  // 接口配置1
#define UB0_REG_PWR_MGMT0            0x4E  // 电源管理
#define UB0_REG_GYRO_CONFIG0         0x4F  // 陀螺仪配置0
#define UB0_REG_ACCEL_CONFIG0        0x50  // 加速度计配置0
#define UB0_REG_GYRO_CONFIG1         0x51  // 陀螺仪配置1
#define UB0_REG_GYRO_ACCEL_CONFIG0   0x52  // 陀螺仪/加速度计配置0
#define UB0_REG_ACCEFL_CONFIG1       0x53  // 加速度计配置1
#define UB0_REG_TMST_CONFIG          0x54  // 时间戳配置
// break
#define UB0_REG_APEX_CONFIG0     0x56      // APEX配置0
#define UB0_REG_SMD_CONFIG       0x57      // 显著运动检测配置
// break
#define UB0_REG_FIFO_CONFIG1     0x5F      // FIFO配置1
#define UB0_REG_FIFO_CONFIG2     0x60      // FIFO配置2
#define UB0_REG_FIFO_CONFIG3     0x61      // FIFO配置3
#define UB0_REG_FSYNC_CONFIG     0x62      // FSYNC配置
#define UB0_REG_INT_CONFIG0      0x63      // 中断配置0
#define UB0_REG_INT_CONFIG1      0x64      // 中断配置1
#define UB0_REG_INT_SOURCE0      0x65      // 中断源0
#define UB0_REG_INT_SOURCE1      0x66      // 中断源1
// break
#define UB0_REG_INT_SOURCE3      0x68      // 中断源3
#define UB0_REG_INT_SOURCE4      0x69      // 中断源4
// break
#define UB0_REG_FIFO_LOST_PKT0   0x6C      // 丢失数据包数低字节
#define UB0_REG_FIFO_LOST_PKT1   0x6D      // 丢失数据包数高字节
// break
#define UB0_REG_SELF_TEST_CONFIG     0x70  // 自检配置
// break
#define UB0_REG_WHO_AM_I     0x75          // 设备标识寄存器

// ================= User Bank 1 =================
#define UB1_REG_SENSOR_CONFIG0   0x03      // 传感器配置0
// break
#define UB1_REG_GYRO_CONFIG_STATIC2      0x0B  // 陀螺仪静态配置2
#define UB1_REG_GYRO_CONFIG_STATIC3      0x0C  // 陀螺仪静态配置3
#define UB1_REG_GYRO_CONFIG_STATIC4      0x0D  // 陀螺仪静态配置4
#define UB1_REG_GYRO_CONFIG_STATIC5      0x0E  // 陀螺仪静态配置5
#define UB1_REG_GYRO_CONFIG_STATIC6      0x0F  // 陀螺仪静态配置6
#define UB1_REG_GYRO_CONFIG_STATIC7      0x10  // 陀螺仪静态配置7
#define UB1_REG_GYRO_CONFIG_STATIC8      0x11  // 陀螺仪静态配置8
#define UB1_REG_GYRO_CONFIG_STATIC9      0x12  // 陀螺仪静态配置9
#define UB1_REG_GYRO_CONFIG_STATIC10     0x13  // 陀螺仪静态配置10
// break
#define UB1_REG_XG_ST_DATA   0x5F      // X轴陀螺仪自检数据
#define UB1_REG_YG_ST_DATA   0x60      // Y轴陀螺仪自检数据
#define UB1_REG_ZG_ST_DATA   0x61      // Z轴陀螺仪自检数据
#define UB1_REG_TMSTVAL0     0x62      // 时间戳值0
#define UB1_REG_TMSTVAL1     0x63      // 时间戳值1
#define UB1_REG_TMSTVAL2     0x64      // 时间戳值2
// break
#define UB1_REG_INTF_CONFIG4     0x7A  // 接口配置4
#define UB1_REG_INTF_CONFIG5     0x7B  // 接口配置5
#define UB1_REG_INTF_CONFIG6     0x7C  // 接口配置6

// ================= User Bank 2 =================
#define UB2_REG_ACCEL_CONFIG_STATIC2     0x03  // 加速度计静态配置2
#define UB2_REG_ACCEL_CONFIG_STATIC3     0x04  // 加速度计静态配置3
#define UB2_REG_ACCEL_CONFIG_STATIC4     0x05  // 加速度计静态配置4
// break
#define UB2_REG_XA_ST_DATA   0x3B      // X轴加速度计自检数据
#define UB2_REG_YA_ST_DATA   0x3C      // Y轴加速度计自检数据
#define UB2_REG_ZA_ST_DATA   0x3D      // Z轴加速度计自检数据

// ================= User Bank 4 =================
#define UB4_REG_APEX_CONFIG1     0x40      // APEX配置1
#define UB4_REG_APEX_CONFIG2     0x41      // APEX配置2
#define UB4_REG_APEX_CONFIG3     0x42      // APEX配置3
#define UB4_REG_APEX_CONFIG4     0x43      // APEX配置4
#define UB4_REG_APEX_CONFIG5     0x44      // APEX配置5
#define UB4_REG_APEX_CONFIG6     0x45      // APEX配置6
#define UB4_REG_APEX_CONFIG7     0x46      // APEX配置7
#define UB4_REG_APEX_CONFIG8     0x47      // APEX配置8
#define UB4_REG_APEX_CONFIG9     0x48      // APEX配置9
// break
#define UB4_REG_ACCEL_WOM_X_THR      0x4A  // 加速度计运动唤醒X轴阈值
#define UB4_REG_ACCEL_WOM_Y_THR      0x4B  // 加速度计运动唤醒Y轴阈值
#define UB4_REG_ACCEL_WOM_Z_THR      0x4C  // 加速度计运动唤醒Z轴阈值
#define UB4_REG_INT_SOURCE6          0x4D  // 中断源6
#define UB4_REG_INT_SOURCE7          0x4E  // 中断源7
#define UB4_REG_INT_SOURCE8          0x4F  // 中断源8
#define UB4_REG_INT_SOURCE9          0x50  // 中断源9
#define UB4_REG_INT_SOURCE10         0x51  // 中断源10
// break
#define UB4_REG_OFFSET_USER0     0x77      // 用户Offset数据0
#define UB4_REG_OFFSET_USER1     0x78      // 用户Offset数据1
#define UB4_REG_OFFSET_USER2     0x79      // 用户Offset数据2
#define UB4_REG_OFFSET_USER3     0x7A      // 用户Offset数据3
#define UB4_REG_OFFSET_USER4     0x7B      // 用户Offset数据4
#define UB4_REG_OFFSET_USER5     0x7C      // 用户Offset数据5
#define UB4_REG_OFFSET_USER6     0x7D      // 用户Offset数据6
#define UB4_REG_OFFSET_USER7     0x7E      // 用户Offset数据7
#define UB4_REG_OFFSET_USER8     0x7F      // 用户Offset数据8

#define DUMMY_BYTE               0xFF      // 虚拟字节（SPI读取填充）


#endif  // ICM42688_REGISTERS_H_
