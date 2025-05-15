#ifndef TRACT_TASKS_COMMON_H
#define TRACT_TASKS_COMMON_H

// Right motor connected to Driver's OUT2

#define TRACT_CONTROL_STACK_SIZE            4096
#define TRACT_CONTROL_TASK_PRIORITY         20 
#define TRACT_CONTROL_CORE_ID               0

#define BDC_MCPWM_TIMER_RESOLUTION_HZ       10000000 // 10MHz, 1 tick = 0.1us
#define BDC_ENCODER_PCNT_HIGH_LIMIT         2000
#define BDC_ENCODER_PCNT_LOW_LIMIT          -2000
#define BDC_PID_LOOP_PERIOD_MS              10

#define TRACT_MOTORS_MCPWM_GROUP            0
#define TRACT_MOTORS_PWM_FREQ               20000       // 20kHz PWM

#define TRACT_ML_PWMA                       27
#define TRACT_ML_PWMB                       14
#define TRACT_ML_ENCODER_A                  26
#define TRACT_ML_ENCODER_B                  25

#define TRACT_MR_PWMA                        33
#define TRACT_MR_PWMB                        32
#define TRACT_MR_ENCODER_A                   34
#define TRACT_MR_ENCODER_B                   35

#define MOTOR1_ENCODER_RES                  908
#define MOTOR2_ENCODER_RES                  908

#define TRACT_ML_KP                         0.50f
#define TRACT_ML_KI                         0.10f
#define TRACT_ML_KD                         0.50f

#define TRACT_MR_KP                         0.50f
#define TRACT_MR_KI                         0.10f
#define TRACT_MR_KD                         0.50f

// #define MOTOR_LEFT_KP                       1.00f
// #define MOTOR_LEFT_KI                       0.54f
// #define MOTOR_LEFT_KD                       0.7f
// 
// #define MOTOR_RIGHT_KP                      0.70f
// #define MOTOR_RIGHT_KI                      0.30f
// #define MOTOR_RIGHT_KD                      0.20f

/* Originales */
//#define TRACT_ML_KP                         0.65f
//#define TRACT_ML_KI                         0.54f
//#define TRACT_ML_KD                         0.5f
//
//#define TRACT_MR_KP                         0.65f
//#define TRACT_MR_KI                         0.54f
//#define TRACT_MR_KD                         0.5f

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TRACT_MOTOR_MAX_PULSES           550
#define TRACT_MOTOR_MAX_REVS             2.718f

// Fix loosy conversions
#define TRACT_PULSES_PER_REV_10MS        204.0f 
#define TRACITON_TOTAL_PULSES_PER_REV    TRACT_PULSES_PER_REV_10MS * 10.0f 

#define TRACT_PULSES2REV_FACTOR          1/TRACT_PULSES_PER_REV_10MS

// One revolution has 2040 pulses = 2*PI rad
#define TRACT_PULSES2RAD_FACTOR          (2 * M_PI) / (TRACITON_TOTAL_PULSES_PER_REV)

#define TRACT_CONV_REV2PULSES(X)         (X * TRACT_PULSES_PER_REV_10MS)
#define TRACT_CONV_PULSES2REV(X)         (X * TRACT_PULSES2REV_FACTOR)
#define TRACT_CONV_PULSES2RAD(X)         (X * TRACT_PULSES2RAD_FACTOR)

#endif