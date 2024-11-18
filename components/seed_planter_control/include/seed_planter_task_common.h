#ifndef SEED_PLANTER_TASK_COMMON_H
#define SEED_PLANTER_TASK_COMMON_H

#define SEED_PLANTER_STACK_SIZE                     4096
#define SEED_PLANTER_TASK_PRIORITY                  3
#define SEED_PLANTER_CORE_ID                        0

#define SEED_PLANTER_MCPWM_TIMER_RESOLUTION_HZ      10000000 // 10MHz, 1 tick = 0.1us
#define SEED_PLANTER_ENCODER_PCNT_HIGH_LIMIT        2000
#define SEED_PLANTER_ENCODER_PCNT_LOW_LIMIT         -2000
#define SEED_PLANTER_PID_LOOP_PERIOD_MS             10

#define SEED_PLANTER_MCPWM_GROUP                    1
#define SEED_PLANTER_MOTORS_PWM_FREQ                25000       // 25kHz PWM
#define SEED_PLANTER_BDC_ENCODER_PCNT_HIGH_LIMIT    2000
#define SEED_PLANTER_BDC_ENCODER_PCNT_LOW_LIMIT     -2000
#define SEED_PLANTER_PID_LOOP_PERIDO_MS             10

#define CUTTER_DISC_PWMA                            14
#define CUTTER_DISC_PWMB                            27
#define CUTTER_DISC_ENCODER_A                       26
#define CUTTER_DISC_ENCODER_B                       25

#define SEED_DISPENSER_PWMA                         32
#define SEED_DISPENSER_PWMB                         33
#define SEED_DISPENSER_ENCODER_A                    34
#define SEED_DISPENSER_ENCODER_B                    35

#define CUTTER_DISC_KP                              0.4f
#define CUTTER_DISC_KI                              0.6f
#define CUTTER_DISC_KD                              0.8f

#define SEED_DISPENSER_KP                           0.6f
#define SEED_DISPENSER_KI                           0.4f
#define SEED_DISPENSER_KD                           0.2f

#define CUTTER_DISC_MAX_SPEED_REVS                  5.876f
#define SEED_DISPENSER_MAX_SPEED_REVS               1.854f

#define CUTTER_DISC_REV2PULSES                      10 / 1.13
#define SEED_DISPENSER_REV2PULSES                   20 / 1.03
#define CUTTER_DISC_PULSES2REV                      0.113f
#define SEED_DISPENSER_PULSES2REV                   0.0515f 


#endif
