
#ifndef TASKS_COMMON_H
#define TASKS_COMMON_H

// Right motor connected to Driver's OUT2

#define TRACTION_CONTROL_STACK_SIZE         4096
#define TRACTION_CONTROL_TASK_PRIORITY      12 
#define TRACTION_CONTROL_CORE_ID            0

#define BDC_MCPWM_TIMER_RESOLUTION_HZ       10000000 // 10MHz, 1 tick = 0.1us
#define BDC_ENCODER_PCNT_HIGH_LIMIT         2000
#define BDC_ENCODER_PCNT_LOW_LIMIT          -2000
#define BDC_PID_LOOP_PERIOD_MS              10

#define TRACTION_MOTORS_MCPWM_GROUP         0
#define TRACTION_MOTORS_PWM_FREQ            25000       // 25kHz PWM

#define TRACTION_MOTOR_LEFT_PWMA            14
#define TRACTION_MOTOR_LEFT_PWMB            27
#define TRACTION_MOTOR_LEFT_ENCODER_A       26
#define TRACTION_MOTOR_LEFT_ENCODER_B       25

#define TRACTION_MOTOR_RIGHT_PWMA           32
#define TRACTION_MOTOR_RIGHT_PWMB           33
#define TRACTION_MOTOR_RIGHT_ENCODER_A      34
#define TRACTION_MOTOR_RIGHT_ENCODER_B      35

#define MOTOR1_ENCODER_RES                  908
#define MOTOR2_ENCODER_RES                  908

#define TRACTION_MOTOR_LEFT_KP              0.007859181f
#define TRACTION_MOTOR_LEFT_KI              3.785918094f
#define TRACTION_MOTOR_LEFT_KD              0.0000001f

#define TRACTION_MOTOR_RIGHT_KP             0.015922355f
#define TRACTION_MOTOR_RIGHT_KI             2.184470949f
#define TRACTION_MOTOR_RIGHT_KD             0.0f

//#define TRACTION_MOTOR_LEFT_KP              0.6 // 6.4
//#define TRACTION_MOTOR_LEFT_KI              0.4
//#define TRACTION_MOTOR_LEFT_KD              0.2 
//
//#define TRACTION_MOTOR_RIGHT_KP             0.6
//#define TRACTION_MOTOR_RIGHT_KI             0.4
//#define TRACTION_MOTOR_RIGHT_KD             0.2

#define TRACTION_ML_MAX_PULSES              18
#define TRACTION_MR_MAX_PULSES              40

#define TRACTION_M_LEFT_MAX_SPEED_REVS      6.0f
#define TRACTION_M_RIGHT_MAX_SPEED_REVS     2.0f

// Fix loosy conversions 
#define TRACTION_M_RIGHT_REV2PULSES         19.8f 
#define TRACTION_M_LEFT_REV2PULSES          9.0f

#define TRACTION_M_RIGHT_PULSES2REV         1/19.8
#define TRACTION_M_LEFT_PULSES2REV          1/9.0

#define TRM_R_PULSES2RAD                    3.14159265 / 990
#define TRM_L_PULSES2RAD                    3.14159265 / 450

// Left motor rev/s to pulses/10mS
#define TRACTION_ML_REV2PULSES(X)    (X * TRACTION_M_LEFT_REV2PULSES)
// Right motor rev/s to pulses/10mS
#define TRACTION_MR_REV2PULSES(Y)   (Y * TRACTION_M_RIGHT_REV2PULSES)
// Left motor pulses to rad 
#define TRACTION_ML_PULSES2RAD(x)    (x * TRM_L_PULSES2RAD)
// Right motor pulses to rad 
#define TRACTION_MR_PULSES2RAD(x)   (x * TRM_R_PULSES2RAD)

#endif