#include "gesture.h"
#include <math.h>


GestureType Gesture_Recognize(MPU6050_Data *data)
{
    float roll = data->roll;
    float pitch = data->pitch;
    float az = data->az;
    
    
    if (fabsf(roll) > EMERGENCY_ANGLE || fabsf(pitch) > EMERGENCY_ANGLE) {
        return GESTURE_EMERGENCY_STOP;
    }
    
    
    if (az > ACCEL_UP_THRESHOLD) {
        return GESTURE_UP;
    }
    
    
    if (az < ACCEL_DOWN_THRESHOLD) {
        return GESTURE_DOWN;
    }
    
    
    if (pitch > ANGLE_THRESHOLD && fabsf(roll) < ANGLE_THRESHOLD) {
        return GESTURE_FORWARD;
    }
    
    
    if (pitch < -ANGLE_THRESHOLD && fabsf(roll) < ANGLE_THRESHOLD) {
        return GESTURE_BACKWARD;
    }
    
    
    if (roll < -ANGLE_THRESHOLD && fabsf(pitch) < ANGLE_THRESHOLD) {
        return GESTURE_LEFT;
    }
    
    
    if (roll > ANGLE_THRESHOLD && fabsf(pitch) < ANGLE_THRESHOLD) {
        return GESTURE_RIGHT;
    }
    
    
    if (fabsf(roll) < HOVER_TOLERANCE && fabsf(pitch) < HOVER_TOLERANCE) {
        return GESTURE_HOVER;
    }
    
    return GESTURE_NONE;
}


const char* Gesture_GetName(GestureType gesture)
{
    switch (gesture) {
        case GESTURE_HOVER:          return "Hover";
        case GESTURE_FORWARD:        return "Forward";
        case GESTURE_BACKWARD:       return "Backward";
        case GESTURE_LEFT:           return "Left";
        case GESTURE_RIGHT:          return "Right";
        case GESTURE_UP:             return "Up";
        case GESTURE_DOWN:           return "Down";
        case GESTURE_EMERGENCY_STOP: return "EMERGENCY";
        default:                     return "None";
    }
}
