#include "platform.h"

#ifdef USE_RANGEFINDER_VL53L1X

#include <stdint.h>
#include "common/time.h"
#include "common/maths.h"
#include "drivers/time.h"
#include "sensors/rangefinder.h"

#include "flight/obstacle_avoidance.h"

#include "drivers/rangefinder/rangefinder_vl53l1x.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "flight/mixer.h"
#include "flight/failsafe.h"
#include "build/debug.h"

#define VL53L1X_MIN_VALID_CM  (5)
#define VL53L1X_MAX_VALID_CM  (280)
#define STATE_DEBOUNCE_MS     (200)

#define RC_NEUTRAL            1500
#define AVOIDANCE_PITCH_REVERSE   100

obstacleAvoidanceConfig_t obstacleAvoidanceConfig = {
    .safeDistanceCm = 50,
    .warningDistanceCm = 100,
    .reverseSpeedCmS = 20,
    .enabled = 1
};

obstacleAvoidanceResult_t obstacleAvoidanceResult;

static int32_t sensorDistances[VL53L1X_IDX_COUNT];
static bool sensorsInitialized = false;
static timeMs_t lastMoveTime = 0;
static timeMs_t safeDistanceEnterTime = 0;
static obstacleAvoidanceState_e desiredState = OBSTACLE_AVOIDANCE_IDLE;
static timeMs_t stateChangeTime = 0;

/**
 * @brief 检查距离数据是否有效
 * 排除：0、负数、超出量程的值
 */
static bool isDistanceValid(int32_t distance)
{
    return (distance >= VL53L1X_MIN_VALID_CM && 
            distance <= VL53L1X_MAX_VALID_CM &&
            distance != RANGEFINDER_OUT_OF_RANGE &&
            distance != 0);
}

static void setStateWithDebounce(obstacleAvoidanceState_e newState) {
    if (desiredState != newState) {
        desiredState = newState;
        stateChangeTime = millis();
    } else if (millis() - stateChangeTime >= STATE_DEBOUNCE_MS) {
        obstacleAvoidanceResult.state = newState;
    }
}

static int32_t getSensorDistance(uint8_t index) {
    if (index >= VL53L1X_IDX_COUNT) {
        return RANGEFINDER_OUT_OF_RANGE;
    }
    int32_t distance = debug[index];
    if (distance < VL53L1X_MIN_VALID_CM || distance > VL53L1X_MAX_VALID_CM) {
        return RANGEFINDER_OUT_OF_RANGE;
    }
    return distance;
}

/**
 * @brief 初始化避障模块
 */
void obstacleAvoidanceInit(void)
{
    debugMode = DEBUG_ALWAYS;
    
    for (int i = 0; i < VL53L1X_IDX_COUNT; i++) {
        sensorDistances[i] = RANGEFINDER_OUT_OF_RANGE;
        obstacleAvoidanceResult.distanceCm[i] = RANGEFINDER_OUT_OF_RANGE;
    }
    
    obstacleAvoidanceResult.state = OBSTACLE_AVOIDANCE_IDLE;
    desiredState = OBSTACLE_AVOIDANCE_IDLE;
    stateChangeTime = millis();
    obstacleAvoidanceResult.obstacleDetected = false;
    obstacleAvoidanceResult.frontObstacle = false;
    obstacleAvoidanceResult.rearObstacle = false;
    obstacleAvoidanceResult.leftObstacle = false;
    obstacleAvoidanceResult.rightObstacle = false;
    
    lastMoveTime = millis();
    sensorsInitialized = true;
}

/**
 * @brief 更新避障状态
 */
void obstacleAvoidanceUpdate(void)
{
    if (!obstacleAvoidanceConfig.enabled || !sensorsInitialized) {
        obstacleAvoidanceResult.state = OBSTACLE_AVOIDANCE_IDLE;
        return;
    }

    timeMs_t now = millis();
    
    // 读取前方两个传感器的数据
    int32_t frontLeftDist = getSensorDistance(VL53L1X_IDX_FRONT_LEFT);
    int32_t frontRightDist = getSensorDistance(VL53L1X_IDX_FRONT_RIGHT);
    
    // 计算前方最小距离
    int32_t frontMinDist = INT32_MAX;
    if (isDistanceValid(frontLeftDist)) {
        frontMinDist = frontLeftDist;
    }
    if (isDistanceValid(frontRightDist) && frontRightDist < frontMinDist) {
        frontMinDist = frontRightDist;
    }
    
    // 更新传感器距离结果
    sensorDistances[VL53L1X_IDX_FRONT_LEFT] = frontLeftDist;
    sensorDistances[VL53L1X_IDX_FRONT_RIGHT] = frontRightDist;
    obstacleAvoidanceResult.distanceCm[VL53L1X_IDX_FRONT_LEFT] = frontLeftDist;
    obstacleAvoidanceResult.distanceCm[VL53L1X_IDX_FRONT_RIGHT] = frontRightDist;
    
    // 检测前方障碍物
    obstacleAvoidanceResult.frontObstacle = 
        (frontMinDist != INT32_MAX && frontMinDist < obstacleAvoidanceConfig.safeDistanceCm);
    
    obstacleAvoidanceResult.obstacleDetected = obstacleAvoidanceResult.frontObstacle;
    
    // 状态机更新
    switch (obstacleAvoidanceResult.state) {
        case OBSTACLE_AVOIDANCE_IDLE:
            if (obstacleAvoidanceResult.frontObstacle) {
                setStateWithDebounce(OBSTACLE_AVOIDANCE_STOPPED);
                lastMoveTime = now;
            }
            break;

        case OBSTACLE_AVOIDANCE_STOPPED:
            if (!obstacleAvoidanceResult.frontObstacle) {
                setStateWithDebounce(OBSTACLE_AVOIDANCE_IDLE);
            }
            else if (now - lastMoveTime > 500) {
                setStateWithDebounce(OBSTACLE_AVOIDANCE_REVERSING);
                lastMoveTime = now;
                safeDistanceEnterTime = 0;
            }
            break;

        case OBSTACLE_AVOIDANCE_REVERSING:
            if (!obstacleAvoidanceResult.frontObstacle) {
                setStateWithDebounce(OBSTACLE_AVOIDANCE_STOPPED);
                lastMoveTime = now;
                safeDistanceEnterTime = 0;
            }
            else if (frontMinDist >= (int32_t)(obstacleAvoidanceConfig.safeDistanceCm * 1.5f)) {
                if (safeDistanceEnterTime == 0) {
                    safeDistanceEnterTime = now;
                }
                else if (now - safeDistanceEnterTime > 200) {
                    setStateWithDebounce(OBSTACLE_AVOIDANCE_STOPPED);
                    lastMoveTime = now;
                    safeDistanceEnterTime = 0;
                }
            }
            else {
                safeDistanceEnterTime = 0;
            }
            break;

        default:
            break;
    }
}

bool obstacleAvoidanceIsActive(void)
{
    return obstacleAvoidanceResult.state != OBSTACLE_AVOIDANCE_IDLE;
}

void obstacleAvoidanceAdjustVelocity(float *velX, float *velY, float cosYaw, float sinYaw)
{
    if (!obstacleAvoidanceConfig.enabled) {
        return;
    }

    float avoidVelX = 0.0f;
    float avoidVelY = 0.0f;

    switch (obstacleAvoidanceResult.state) {
        case OBSTACLE_AVOIDANCE_STOPPED:
            avoidVelX = -(*velX);
            avoidVelY = -(*velY);
            break;

        case OBSTACLE_AVOIDANCE_REVERSING:
            {
                float avoidSpeed = obstacleAvoidanceConfig.reverseSpeedCmS;
                avoidVelX = -avoidSpeed * cosYaw;
                avoidVelY = -avoidSpeed * sinYaw;
            }
            break;

        case OBSTACLE_AVOIDANCE_IDLE:
        default:
            if (obstacleAvoidanceResult.frontObstacle) {
                float velForward = *velX * cosYaw + *velY * sinYaw;
                if (velForward > 0.0f) {
                    *velX = 0.0f;
                    *velY = 0.0f;
                    return;
                }
            }
            return;
    }

    *velX += avoidVelX;
    *velY += avoidVelY;

    float combinedSpeed = calc_length_pythagorean_2D(*velX, *velY);
    if (combinedSpeed > navConfig()->general.max_manual_speed) {
        float scale = navConfig()->general.max_manual_speed / combinedSpeed;
        *velX *= scale;
        *velY *= scale;
    }
}

int32_t obstacleAvoidanceGetDistance(vl53l1xSensorIndex_e index)
{
    if (index >= VL53L1X_IDX_COUNT) {
        return RANGEFINDER_OUT_OF_RANGE;
    }
    return sensorDistances[index];
}

#endif // USE_RANGEFINDER_VL53L1X