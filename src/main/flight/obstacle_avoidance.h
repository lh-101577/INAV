/*
 * INAV避障模块 - 使用6个VL53L1X传感器
 * 传感器布局:
 * - 前方左侧 (VL53L1X_IDX_FRONT_LEFT)
 * - 前方右侧 (VL53L1X_IDX_FRONT_RIGHT)
 * - 后方左侧 (VL53L1X_IDX_REAR_LEFT)
 * - 后方右侧 (VL53L1X_IDX_REAR_RIGHT)
 * - 左侧 (VL53L1X_IDX_LEFT)
 * - 右侧 (VL53L1X_IDX_RIGHT)
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

// 6个VL53L1X传感器的索引定义 (对应debug通道)
typedef enum {
    VL53L1X_IDX_REAR_LEFT = 0,    // debug0 - 后方左侧传感器
    VL53L1X_IDX_REAR_RIGHT,       // debug1 - 后方右侧传感器
    VL53L1X_IDX_RIGHT,            // debug2 - 右侧传感器
    VL53L1X_IDX_LEFT,             // debug3 - 左侧传感器
    VL53L1X_IDX_FRONT_LEFT,       // debug4 - 前方左侧传感器
    VL53L1X_IDX_FRONT_RIGHT,      // debug5 - 前方右侧传感器
    VL53L1X_IDX_COUNT             // 传感器总数
} vl53l1xSensorIndex_e;

// 避障状态枚举
typedef enum {
    OBSTACLE_AVOIDANCE_IDLE = 0,      // 空闲状态 - 未检测到障碍物
    OBSTACLE_AVOIDANCE_STOPPED,       // 停止状态 - 因障碍物停止
    OBSTACLE_AVOIDANCE_REVERSING,     // 后退状态 - 远离前方障碍
    OBSTACLE_AVOIDANCE_MOVING_FORWARD, // 前进状态 - 远离后方障碍
    OBSTACLE_AVOIDANCE_MOVING_LEFT,    // 左移状态 - 远离右侧障碍
    OBSTACLE_AVOIDANCE_MOVING_RIGHT,   // 右移状态 - 远离左侧障碍
    OBSTACLE_AVOIDANCE_WARNING        // 警告状态 - 接近障碍物
} obstacleAvoidanceState_e;

// 障碍物检测结果结构体
typedef struct {
    int32_t distanceCm[VL53L1X_IDX_COUNT];  // 每个传感器的距离(厘米)
    bool obstacleDetected;                    // 是否检测到障碍物
    bool frontObstacle;                       // 前方是否有障碍物
    bool rearObstacle;                        // 后方是否有障碍物
    bool leftObstacle;                        // 左侧是否有障碍物
    bool rightObstacle;                       // 右侧是否有障碍物
    obstacleAvoidanceState_e state;           // 当前避障状态
} obstacleAvoidanceResult_t;

// 避障配置结构体
typedef struct {
    uint16_t safeDistanceCm;          // 安全距离(厘米)
    uint16_t warningDistanceCm;       // 警告距离(厘米)
    uint16_t reverseSpeedCmS;         // 避障后退速度(厘米/秒)
    uint8_t  enabled;                 // 是否启用避障功能
} obstacleAvoidanceConfig_t;

// 全局变量
extern obstacleAvoidanceConfig_t obstacleAvoidanceConfig;  // 避障配置
extern obstacleAvoidanceResult_t obstacleAvoidanceResult;   // 避障结果

// 函数声明
void obstacleAvoidanceInit(void);                           // 初始化避障模块
void obstacleAvoidanceUpdate(void);                         // 更新避障状态
bool obstacleAvoidanceIsActive(void);                       // 检查避障是否激活
void obstacleAvoidanceAdjustVelocity(float *velX, float *velY, float cosYaw, float sinYaw); // 调整导航速度指令
int32_t obstacleAvoidanceGetDistance(vl53l1xSensorIndex_e index); // 获取传感器距离