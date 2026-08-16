#include "gesture_mapper.h"
#include <math.h>

static uint16_t rc_channels[RC_CHANNEL_COUNT];
static uint8_t armed = 0;
static uint32_t last_packet_time = 0;
static uint32_t low_throttle_since = 0;

// 平滑参数
#define HOVER_THROTTLE    1700u   // 悬停油门 = RC_MID + 200（60%）
#define LOW_THROTTLE      1100u   // 下降时的低油门 = RC_MIN + 100
#define THROTTLE_STEP     25u     // 每次更新油门的变化量（100Hz下约2.5s从最低到悬停）
#define ANGLE_RC_STEP     25u     // 每次更新姿态通道的变化量（平滑过渡）

// 限制函数
static float constrain_float(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// 通道值向目标值平滑逼近（斜坡方式，避免阶跃导致飞机抖动）
static void ramp_channel(uint16_t *ch, uint16_t target, uint16_t step)
{
    if (*ch < target) {
        *ch += step;
        if (*ch > target) *ch = target;
    } else if (*ch > target) {
        if (*ch < step) *ch = target;
        else *ch -= step;
    }
}

// 角度映射到RC值
static uint16_t angle_to_rc(float angle)
{
    // 限制角度范围
    angle = constrain_float(angle, -MAX_ANGLE, MAX_ANGLE);
    
    // 映射到RC范围：-45° → 1000, 0° → 1500, +45° → 2000
    int16_t rc_value = RC_MID + (int16_t)(angle * ANGLE_TO_RC_SCALE);
    
    // 限制RC值范围
    if (rc_value < RC_MIN) rc_value = RC_MIN;
    if (rc_value > RC_MAX) rc_value = RC_MAX;
    
    return (uint16_t)rc_value;
}

// 初始化
void GestureMapper_Init(void)
{
    // 初始化所有通道为中位
    for (int i = 0; i < RC_CHANNEL_COUNT; i++) {
        rc_channels[i] = RC_MID;
    }
    
    // 油门设为最低
    rc_channels[RC_THROTTLE] = RC_MIN;
    
    // AUX1（ARM开关）设为低位（未解锁）
    rc_channels[RC_AUX1] = RC_MIN;
    
    // AUX2（飞行模式）设为中位（Angle模式）
    rc_channels[RC_AUX2] = RC_MID;
    
    armed = 0;
}

// 更新RC通道值
void GestureMapper_Update(NRF_DataPacket *packet)
{
    last_packet_time = HAL_GetTick();
    
    GestureType gesture = (GestureType)packet->gesture;
    
    // 根据手势类型处理
    switch (gesture) {
        case GESTURE_EMERGENCY_STOP:
            // 紧急停止：立即上锁、油门归零、姿态回中
            armed = 0;
            rc_channels[RC_AUX1] = RC_MIN;
            rc_channels[RC_THROTTLE] = RC_MIN;
            rc_channels[RC_ROLL] = RC_MID;
            rc_channels[RC_PITCH] = RC_MID;
            rc_channels[RC_YAW] = RC_MID;
            break;
            
        case GESTURE_UP:
            // 起飞/上升
            if (!armed) {
                // 重要：先解锁，油门保持最低。
                // Betaflight默认要求"油门在最低"才允许解锁，
                // 如果解锁的同时给高油门，飞控会拒绝解锁，表现为没反应。
                rc_channels[RC_AUX1] = RC_MAX;
                rc_channels[RC_THROTTLE] = RC_MIN;
                armed = 1;
            } else {
                // 已解锁后油门平滑升至悬停点
                ramp_channel(&rc_channels[RC_THROTTLE], HOVER_THROTTLE, THROTTLE_STEP);
            }
            rc_channels[RC_ROLL] = RC_MID;
            rc_channels[RC_PITCH] = RC_MID;
            rc_channels[RC_YAW] = RC_MID;
            break;
            
        case GESTURE_DOWN:
            // 下降/降落：油门平滑降到低油门
            ramp_channel(&rc_channels[RC_THROTTLE], LOW_THROTTLE, THROTTLE_STEP);
            rc_channels[RC_ROLL] = RC_MID;
            rc_channels[RC_PITCH] = RC_MID;
            rc_channels[RC_YAW] = RC_MID;
            if (armed && rc_channels[RC_THROTTLE] <= LOW_THROTTLE) {
                // 保持低油门1秒后上锁，防止误触瞬间落地
                if (HAL_GetTick() - low_throttle_since > 1000) {
                    armed = 0;
                    rc_channels[RC_AUX1] = RC_MIN;
                    rc_channels[RC_THROTTLE] = RC_MIN;
                }
            } else {
                low_throttle_since = HAL_GetTick();
            }
            break;
            
        case GESTURE_HOVER:
            // 悬停：姿态平滑回中，油门保持当前值
            ramp_channel(&rc_channels[RC_ROLL],  RC_MID, ANGLE_RC_STEP);
            ramp_channel(&rc_channels[RC_PITCH], RC_MID, ANGLE_RC_STEP);
            rc_channels[RC_YAW] = RC_MID;
            break;
            
        case GESTURE_FORWARD:
        case GESTURE_BACKWARD:
        case GESTURE_LEFT:
        case GESTURE_RIGHT:
            // 方向控制：使用姿态角映射并平滑过渡
            ramp_channel(&rc_channels[RC_ROLL],
                         angle_to_rc(packet->roll), ANGLE_RC_STEP);
            ramp_channel(&rc_channels[RC_PITCH],
                         angle_to_rc(packet->pitch), ANGLE_RC_STEP);
            rc_channels[RC_YAW] = RC_MID;  // 偏航保持中位
            break;
            
        default:
            // 无手势：姿态平滑回中，油门保持
            ramp_channel(&rc_channels[RC_ROLL],  RC_MID, ANGLE_RC_STEP);
            ramp_channel(&rc_channels[RC_PITCH], RC_MID, ANGLE_RC_STEP);
            rc_channels[RC_YAW] = RC_MID;
            break;
    }
}

// 获取RC通道数组
void GestureMapper_GetRCChannels(uint16_t *channels)
{
    // 检查信号丢失（超过500ms没收到数据）
    if (HAL_GetTick() - last_packet_time > 500) {
        // 信号丢失：立即断油上锁（失控保护）
        channels[RC_ROLL] = RC_MID;
        channels[RC_PITCH] = RC_MID;
        channels[RC_THROTTLE] = RC_MIN;  // 油门最低
        channels[RC_YAW] = RC_MID;
        channels[RC_AUX1] = RC_MIN;      // 上锁
        channels[RC_AUX2] = RC_MID;
        channels[RC_AUX3] = RC_MID;
        channels[RC_AUX4] = RC_MID;
        armed = 0;
    } else {
        // 正常：复制通道值
        for (int i = 0; i < RC_CHANNEL_COUNT; i++) {
            channels[i] = rc_channels[i];
        }
    }
}

// 获取解锁状态
uint8_t GestureMapper_IsArmed(void)
{
    return armed;
}
