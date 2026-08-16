#ifndef __GESTURE_H
#define __GESTURE_H

#include "mpu6050.h"


typedef enum {
    GESTURE_NONE = 0,       
    GESTURE_HOVER,         
    GESTURE_FORWARD,       
    GESTURE_BACKWARD,      
    GESTURE_LEFT,          
    GESTURE_RIGHT,         
    GESTURE_UP,            
    GESTURE_DOWN,          
    GESTURE_EMERGENCY_STOP  
} GestureType;


#define ANGLE_THRESHOLD     15.0f   
#define ACCEL_UP_THRESHOLD   1.5f   
#define ACCEL_DOWN_THRESHOLD -1.5f  
#define HOVER_TOLERANCE      5.0f   
#define EMERGENCY_ANGLE      60.0f  


GestureType Gesture_Recognize(MPU6050_Data *data);
const char* Gesture_GetName(GestureType gesture);

#endif
