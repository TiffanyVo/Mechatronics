#include"BOARD.h"
#include <xc.h>
#include <stdio.h>
#include <proc/p32mx320f128l.h>
#include "pwm.h"
#include "MotorControl.h"

#define TICKS_DEGREES 26
#define TICKS_INCHES 250

int Motor_Init(void) {
    TRIS_RIGHT_FRONT_FORWARD = 0;
    TRIS_RIGHT_FRONT_REVERSE = 0;
    TRIS_RIGHT_REAR_FORWARD = 0;
    TRIS_RIGHT_REAR_REVERSE = 0;

    TRIS_LEFT_FRONT_FORWARD = 0;
    TRIS_LEFT_FRONT_REVERSE = 0;
    TRIS_LEFT_REAR_FORWARD = 0;
    TRIS_LEFT_REAR_REVERSE = 0;

    PWM_SetFrequency(PWM_500HZ);
    PWM_AddPins(RIGHT_FRONT_ENABLE);
    PWM_AddPins(RIGHT_REAR_ENABLE);
    PWM_AddPins(LEFT_FRONT_ENABLE);
    PWM_AddPins(LEFT_REAR_ENABLE);
    
    TRIS_FRONTLEFT_A = 1;  
    TRIS_FRONTLEFT_B = 1;  
    TRIS_FRONTRIGHT_A = 1;  
    TRIS_FRONTRIGHT_B = 1;  
    TRIS_REARLEFT_A = 1;   
    TRIS_REARLEFT_B = 1;  
    TRIS_REARRIGHT_A = 1; 
    TRIS_REARRIGHT_B = 1; 

    return SUCCESS;
}

int Motor_Reset(void) {
    RIGHT_FRONT_FORWARD = 0;
    RIGHT_FRONT_REVERSE = 0;
    RIGHT_REAR_FORWARD = 0;
    RIGHT_REAR_REVERSE = 0;
    LEFT_FRONT_FORWARD = 0;
    LEFT_FRONT_REVERSE = 0;
    LEFT_REAR_FORWARD = 0;
    LEFT_REAR_REVERSE = 0;

    return SUCCESS;
}

int Motor_Forward(void) {
    RIGHT_FRONT_FORWARD = 1;
    RIGHT_FRONT_REVERSE = 0;
    RIGHT_REAR_FORWARD = 1;
    RIGHT_REAR_REVERSE = 0;
    LEFT_FRONT_FORWARD = 1;
    LEFT_FRONT_REVERSE = 0;
    LEFT_REAR_FORWARD = 1;
    LEFT_REAR_REVERSE = 0;
    return SUCCESS;
}

int Motor_Reverse(void) {
    RIGHT_FRONT_FORWARD = 0;
    RIGHT_FRONT_REVERSE = 1;
    RIGHT_REAR_FORWARD = 0;
    RIGHT_REAR_REVERSE = 1;
    LEFT_FRONT_FORWARD = 0;
    LEFT_FRONT_REVERSE = 1;
    LEFT_REAR_FORWARD = 0;
    LEFT_REAR_REVERSE = 1;
    return SUCCESS;
}

int Motor_StrafeRight(void) {
    RIGHT_FRONT_FORWARD = 0;
    RIGHT_FRONT_REVERSE = 1;
    RIGHT_REAR_FORWARD = 1;
    RIGHT_REAR_REVERSE = 0;
    LEFT_FRONT_FORWARD = 1;
    LEFT_FRONT_REVERSE = 0;
    LEFT_REAR_FORWARD = 0;
    LEFT_REAR_REVERSE = 1;
    return SUCCESS;
}

int Motor_StrafeLeft(void) {
    RIGHT_FRONT_FORWARD = 1;
    RIGHT_FRONT_REVERSE = 0;
    RIGHT_REAR_FORWARD = 0;
    RIGHT_REAR_REVERSE = 1;
    LEFT_FRONT_FORWARD = 0;
    LEFT_FRONT_REVERSE = 1;
    LEFT_REAR_FORWARD = 1;
    LEFT_REAR_REVERSE = 0;
    return SUCCESS;
}

int Motor_TankLeft(void) {
    RIGHT_FRONT_FORWARD = 1;
    RIGHT_FRONT_REVERSE = 0;
    RIGHT_REAR_FORWARD = 1;
    RIGHT_REAR_REVERSE = 0;
    LEFT_FRONT_FORWARD = 0;
    LEFT_FRONT_REVERSE = 1;
    LEFT_REAR_FORWARD = 0;
    LEFT_REAR_REVERSE = 1;
    return SUCCESS;
}

int Motor_TankRight(void) {
    RIGHT_FRONT_FORWARD = 0;
    RIGHT_FRONT_REVERSE = 1;
    RIGHT_REAR_FORWARD = 0;
    RIGHT_REAR_REVERSE = 1;
    LEFT_FRONT_FORWARD = 1;
    LEFT_FRONT_REVERSE = 0;
    LEFT_REAR_FORWARD = 1;
    LEFT_REAR_REVERSE = 0;
    return SUCCESS;
}

int Motor_AllDutyCycle(unsigned int Duty) {
    PWM_SetDutyCycle(RIGHT_FRONT_ENABLE, Duty);
    PWM_SetDutyCycle(RIGHT_REAR_ENABLE, Duty);
    PWM_SetDutyCycle(LEFT_FRONT_ENABLE, Duty);
    PWM_SetDutyCycle(LEFT_REAR_ENABLE, Duty);
}

int Motor_SlowTurn(unsigned int leftDutyCycle, unsigned int rightDutyCycle) {
    PWM_SetDutyCycle(RIGHT_FRONT_ENABLE, rightDutyCycle);
    PWM_SetDutyCycle(RIGHT_REAR_ENABLE, rightDutyCycle);
    PWM_SetDutyCycle(LEFT_FRONT_ENABLE, leftDutyCycle);
    PWM_SetDutyCycle(LEFT_REAR_ENABLE, leftDutyCycle);
}

int Motor_SetIndividual(unsigned int FRDutyCycle, unsigned int FLDutyCycle, unsigned int RLDutyCycle, unsigned int RRDutyCycle) {
    PWM_SetDutyCycle(RIGHT_FRONT_ENABLE, FRDutyCycle);
    PWM_SetDutyCycle(RIGHT_REAR_ENABLE, RRDutyCycle);
    PWM_SetDutyCycle(LEFT_FRONT_ENABLE, FLDutyCycle);
    PWM_SetDutyCycle(LEFT_REAR_ENABLE, RLDutyCycle);
}

int InchesToTicks(int inches){
    return inches*TICKS_INCHES;
}

int DegreesToTicks(int degrees){
    return degrees*TICKS_DEGREES;
}