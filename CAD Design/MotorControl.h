#include"BOARD.h"
#include <xc.h>
#include <stdio.h>
#include <proc/p32mx320f128l.h>
#include "pwm.h"
#include "IO_Ports.h"

#define TRIS_RIGHT_FRONT_FORWARD PORTZ07_TRIS         // PORT Z 7
#define TRIS_RIGHT_FRONT_REVERSE PORTZ04_TRIS         // PORT Z 4
#define TRIS_RIGHT_REAR_FORWARD PORTZ05_TRIS          // PORT Z 5
#define TRIS_RIGHT_REAR_REVERSE PORTZ03_TRIS          // PORT Z 3

#define TRIS_LEFT_REAR_FORWARD PORTX09_TRIS           // PORT X 9
#define TRIS_LEFT_REAR_REVERSE PORTX11_TRIS           // PORT X 11
#define TRIS_LEFT_FRONT_FORWARD PORTX10_TRIS          // PORT X 10
#define TRIS_LEFT_FRONT_REVERSE PORTX12_TRIS          // PORT X 12

#define LEFT_REAR_FORWARD PORTX09_LAT                 // PORT X 9
#define LEFT_FRONT_FORWARD PORTX10_LAT                // PORT X 10
#define LEFT_FRONT_REVERSE PORTX12_LAT                // PORT X 12
#define LEFT_REAR_REVERSE PORTX11_LAT                 // PORT X 11

#define RIGHT_FRONT_FORWARD PORTZ07_LAT               // PORT Z 7
#define RIGHT_FRONT_REVERSE PORTZ04_LAT               // PORT Z 4
#define RIGHT_REAR_FORWARD PORTZ05_LAT                // PORT Z 5
#define RIGHT_REAR_REVERSE PORTZ03_LAT                // PORT Z 3

#define TRIS_RIGHT_FRONT_ENABLE TRISDbits.TRISD2      // PORT Y 10
#define TRIS_RIGHT_REAR_ENABLE  TRISDbits.TRISD3      // PORT Y 4
#define TRIS_LEFT_REAR_ENABLE   TRISDbits.TRISD1      // PORT Y 12
#define TRIS_LEFT_FRONT_ENABLE  TRISDbits.TRISD0      // PORT Z 6

#define RIGHT_FRONT_ENABLE PWM_PORTY10 
#define RIGHT_REAR_ENABLE PWM_PORTY04 
#define LEFT_FRONT_ENABLE PWM_PORTZ06
#define LEFT_REAR_ENABLE PWM_PORTY12

#define TRIS_FRONTLEFT_A PORTX05_TRIS                 // PORT X 5
#define TRIS_FRONTLEFT_B PORTX06_TRIS                 // PORT X 6
#define TRIS_FRONTRIGHT_A PORTZ08_TRIS                // PORT Z 8
#define TRIS_FRONTRIGHT_B PORTZ09_TRIS                // PORT Z 9
#define TRIS_REARLEFT_A PORTX07_TRIS                  // PORT X 7
#define TRIS_REARLEFT_B PORTX08_TRIS                  // PORT X 8
#define TRIS_REARRIGHT_A PORTX03_TRIS                 // PORT X 3
#define TRIS_REARRIGHT_B PORTX04_TRIS                 // PORT X 4

int Motor_Init(void);

int Motor_Reset(void);

int Motor_Forward(void);

int Motor_Reverse(void);

int Motor_StrafeRight(void);

int Motor_StrafeLeft(void);

int Motor_TankLeft(void);

int Motor_TankRight(void);

int Motor_AllDutyCycle(unsigned int Duty);

int Motor_SlowTurn(unsigned int leftDutyCycle, unsigned int rightDutyCycle);
int Motor_SetIndividual(unsigned int FRDutyCycle, unsigned int FLDutyCycle, unsigned int RLDutyCycle, unsigned int RRDutyCycle);

int InchesToTicks(int inches);

int DegreesToTicks(int degrees);