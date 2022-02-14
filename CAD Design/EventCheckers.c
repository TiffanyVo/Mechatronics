/*
 * File:   TemplateEventChecker.c
 * Author: Gabriel Hugh Elkaim
 *
 * Template file to set up typical EventCheckers for the  Events and Services
 * Framework (ES_Framework) on the Uno32 for the CMPE-118/L class. Note that
 * this file will need to be modified to fit your exact needs, and most of the
 * names will have to be changed to match your code.
 *
 * This EventCheckers file will work with both FSM's and HSM's.
 *
 * Remember that EventCheckers should only return TRUE when an event has occured,
 * and that the event is a TRANSITION between two detectable differences. They
 * should also be atomic and run as fast as possible for good results.
 *
 * This file includes a test harness that will run the event detectors listed in the
 * ES_Configure file in the project, and will conditionally compile main if the macro
 * EVENTCHECKER_TEST is defined (either in the project or in the file). This will allow
 * you to check you event detectors in their own project, and then leave them untouched
 * for your project unless you need to alter their post functions.
 *
 * Created on September 27, 2013, 8:37 AM
 */

/*******************************************************************************
 * MODULE #INCLUDE                                                             *
 ******************************************************************************/

#include <proc/p32mx320f128l.h>

#include "ES_Configure.h"
#include "EventCheckers.h"
#include "ES_Events.h"
#include "serial.h"
#include "AD.h"
#include "EncoderService.h"
#include "ES_CheckEvents.h"
#include "PingSensorService.h"
#include "timers.h"
#include "OrientationFSM.h"
#include "math.h"
#include "TopLvlHSM.h"

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/
#define BATTERY_DISCONNECT_THRESHOLD 175
#define FRONTLEFT_A PORTX05_BIT // PORT X 5
#define FRONTLEFT_B PORTX06_BIT // PORT X 6
#define FRONTRIGHT_A PORTZ08_BIT // PORT Z 8
#define FRONTRIGHT_B PORTZ09_BIT // PORT Z 9
#define REARLEFT_A PORTX07_BIT // PORT X 7
#define REARLEFT_B PORTX08_BIT // PORT X 8
#define REARRIGHT_A PORTX03_BIT // PORT V 6
#define REARRIGHT_B PORTX04_BIT // PORT V 5

#define TIMER1_uS_PER_TICK 25
#define CUR_TIME TIMERS_GetTime()*1000 + TMR1*TIMER1_uS_PER_TICK

/*******************************************************************************
 * EVENTCHECKER_TEST SPECIFIC CODE                                                             *
 ******************************************************************************/

//#define EVENTCHECKER_TEST
#ifdef EVENTCHECKER_TEST
#include <stdio.h>
#define SaveEvent(x) do {eventName=__func__; storedEvent=x;} while (0)

static const char *eventName;
static ES_Event storedEvent;
#endif

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES                                                 *
 ******************************************************************************/
/* Prototypes for private functions for this EventChecker. They should be functions
   relevant to the behavior of this particular event checker */

int GetAllWheelAvg(void);
int GetLeftBankAvg(void);
int GetRightBankAvg(void);

/*******************************************************************************
 * PRIVATE MODULE VARIABLES                                                    *
 ******************************************************************************/
typedef struct MotorVar {
    uint8_t ABLast;
    uint8_t ABCur;
} MotorVar;

static MotorVar FrontLeft;
static MotorVar FrontRight;
static MotorVar RearLeft;
static MotorVar RearRight;

static int AllWheelAvg;
static int RightBankAvg;
static int LeftBankAvg;

typedef enum MotorStates {
    State00, State10, State11, State01
} MotorStates;

typedef struct Motors {
    int32_t count;
    MotorStates states;
} Motors;

static uint8_t MyPriority;
static Motors FrontLeftState;
static Motors FrontRightState;
static Motors RearRightState;
static Motors RearLeftState;

static uint8_t initFlag = 0;

int32_t Motor_FRQuadCount(void);
int32_t Motor_FLQuadCount(void);
int32_t Motor_RRQuadCount(void);
int32_t Motor_RLQuadCount(void);

Motors RunMotorSM(Motors MotorInput, uint16_t newAB);

// PingChecker Variables
//typedef enum PingStates {
//    LHigh,
//    LLow,
//    RHigh,
//    RLow,
//} PingStates;
//
//static int lastLeftTime;
//static int lastRightTime;
//static int lastLeftDistance;
//static int lastRightDistance;

//moveAmount Checker Variables

static MovementStates moveDirection;
static uint8_t moveActive = FALSE;
static int32_t moveAmount = 0;
static uint8_t leftDone = FALSE;
static uint8_t rightDone = FALSE;



/* Any private module level variable that you might need for keeping track of
   events would be placed here. Private variables should be STATIC so that they
   are limited in scope to this module. */

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/

/**
 * @Function TemplateCheckBattery(void)
 * @param none
 * @return TRUE or FALSE
 * @brief This function is a prototype event checker that checks the battery voltage
 *        against a fixed threshold (#defined in the .c file). Note that you need to
 *        keep track of previous history, and that the actual battery voltage is checked
 *        only once at the beginning of the function. The function will post an event
 *        of either BATTERY_CONNECTED or BATTERY_DISCONNECTED if the power switch is turned
 *        on or off with the USB cord plugged into the Uno32. Returns TRUE if there was an 
 *        event, FALSE otherwise.
 * @note Use this code as a template for your other event checkers, and modify as necessary.
 * @author Gabriel H Elkaim, 2013.09.27 09:18
 * @modified Gabriel H Elkaim/Max Dunne, 2016.09.12 20:08 */
uint8_t TemplateCheckBattery(void) {
    static ES_EventTyp_t lastEvent = BATTERY_DISCONNECTED;
    ES_EventTyp_t curEvent;
    ES_Event thisEvent;
    uint8_t returnVal = FALSE;
    uint16_t batVoltage = AD_ReadADPin(BAT_VOLTAGE); // read the battery voltage

    if (batVoltage > BATTERY_DISCONNECT_THRESHOLD) { // is battery connected?
        curEvent = BATTERY_CONNECTED;
    } else {
        curEvent = BATTERY_DISCONNECTED;
    }
    if (curEvent != lastEvent) { // check for change from last time
        thisEvent.EventType = curEvent;
        thisEvent.EventParam = batVoltage;
        returnVal = TRUE;
        lastEvent = curEvent; // update history
#ifndef EVENTCHECKER_TEST           // keep this as is for test harness
        PostHSM(thisEvent);
#else
        SaveEvent(thisEvent);
#endif   
    }
    return (returnVal);
}

uint8_t CheckMotors(void) {

    if (!initFlag) {
        FrontLeftState.states = State00;
        FrontLeftState.count = 0;
        FrontRightState.states = State00;
        FrontRightState.count = 0;
        RearLeftState.states = State00;
        RearLeftState.count = 0;
        RearRightState.states = State00;
        RearRightState.count = 0;
        initFlag = 1;

    }

    ES_EventTyp_t curEvent;
    ES_Event thisEvent;
    uint8_t returnVal = FALSE;

    FrontLeft.ABCur = (FRONTLEFT_A << 1) + FRONTLEFT_B;
    FrontRight.ABCur = (FRONTRIGHT_A << 1) + FRONTRIGHT_B;
    RearLeft.ABCur = (REARLEFT_A << 1) + REARLEFT_B;
    RearRight.ABCur = (REARRIGHT_A << 1) + REARRIGHT_B;

    if (FrontLeft.ABCur != FrontLeft.ABLast) {
        curEvent = FRONT_LEFT_MOTOR;
        thisEvent.EventType = curEvent;
        thisEvent.EventParam = FrontLeft.ABCur;
        returnVal = TRUE;
        FrontLeft.ABLast = FrontLeft.ABCur; // update history
#ifndef EVENTCHECKER_TEST           // keep this as is for test harness
        FrontLeftState = RunMotorSM(FrontLeftState, thisEvent.EventParam);
#else
        SaveEvent(thisEvent);
#endif   
    }

    if (FrontRight.ABCur != FrontRight.ABLast) {
        curEvent = FRONT_RIGHT_MOTOR;
        thisEvent.EventType = curEvent;
        thisEvent.EventParam = FrontRight.ABCur;
        returnVal = TRUE;
        FrontRight.ABLast = FrontRight.ABCur; // update history
#ifndef EVENTCHECKER_TEST           // keep this as is for test harness
        FrontRightState = RunMotorSM(FrontRightState, thisEvent.EventParam);
#else
        SaveEvent(thisEvent);
#endif   
    }

    if (RearLeft.ABCur != RearLeft.ABLast) {
        curEvent = REAR_LEFT_MOTOR;
        thisEvent.EventType = curEvent;
        thisEvent.EventParam = RearLeft.ABCur;
        returnVal = TRUE;
        RearLeft.ABLast = RearLeft.ABCur; // update history
#ifndef EVENTCHECKER_TEST           // keep this as is for test harness
        RearLeftState = RunMotorSM(RearLeftState, thisEvent.EventParam);
#else
        SaveEvent(thisEvent);
#endif   
    }

    if (RearRight.ABCur != RearRight.ABLast) {
        curEvent = REAR_RIGHT_MOTOR;
        thisEvent.EventType = curEvent;
        thisEvent.EventParam = RearRight.ABCur;
        returnVal = TRUE;
        RearRight.ABLast = RearRight.ABCur; // update history
#ifndef EVENTCHECKER_TEST           // keep this as is for test harness
        RearRightState = RunMotorSM(RearRightState, thisEvent.EventParam);
#else
        SaveEvent(thisEvent);
#endif   
    }
    return (returnVal);
}

void SetMoveAmount(MovementStates direction, int32_t amount) {
    moveDirection = direction;
    moveAmount = amount;
    moveActive = TRUE;

    FrontLeftState.count = 0;
    FrontRightState.count = 0;
    RearLeftState.count = 0;
    RearRightState.count = 0;
    
    if(direction == Turn){
        leftDone = FALSE;
        rightDone = FALSE;
    }
}

void StopMovement(){
    moveActive = FALSE;
}

uint8_t MoveAmount(void) {
    static ES_EventTyp_t lastEvent = ES_NO_EVENT;
    ES_EventTyp_t curEvent;
    ES_Event thisEvent;
    uint8_t returnVal = FALSE;
    
    int32_t averageVal = 0;
    int32_t averageLeft = 0;
    int32_t averageRight = 0;
    int32_t averageFront = 0;
    int32_t averageRear = 0;
    
    if (moveActive) {
        switch (moveDirection) {
            case Move:
                averageVal = (abs(FrontLeftState.count) + abs(FrontRightState.count) + abs(RearLeftState.count) + abs(RearRightState.count)) >> 2; //average encoder value
                if (averageVal > moveAmount) {
                    moveActive = FALSE;
                    curEvent = FINISHED_MOVING;
                    thisEvent.EventType = curEvent;
                    thisEvent.EventParam = moveDirection;
                    returnVal = TRUE;
                    #ifndef EVENTCHECKER_TEST           // keep this as is for test harness
                        PostHSM(thisEvent);
                    #else
                        SaveEvent(thisEvent);
                    #endif
                }
                break;
                
            case Turn:
                averageLeft = (abs(FrontLeftState.count) + abs(RearLeftState.count)) >> 1;
                averageRight = (abs(RearRightState.count) + abs(FrontRightState.count)) >> 1;
                
                if(!leftDone && (averageLeft > moveAmount)){
                    curEvent = FINISHED_MOVING_LEFT_BANK;
                    thisEvent.EventType = curEvent;
                    thisEvent.EventParam = moveDirection;
                    leftDone = TRUE;
                    returnVal = TRUE;
                    #ifndef EVENTCHECKER_TEST           // keep this as is for test harness
                        PostHSM(thisEvent);
                    #else
                        SaveEvent(thisEvent);
                    #endif
                }
                if(!rightDone && (averageRight > moveAmount)){
                    curEvent = FINISHED_MOVING_RIGHT_BANK;
                    thisEvent.EventType = curEvent;
                    thisEvent.EventParam = moveDirection;
                    rightDone = TRUE;
                    returnVal = TRUE;
                    #ifndef EVENTCHECKER_TEST           // keep this as is for test harness
                        PostHSM(thisEvent);
                    #else
                        SaveEvent(thisEvent);
                    #endif
                }
                if(leftDone && rightDone){
                    moveActive = FALSE;
                    curEvent = FINISHED_TURNING;
                    thisEvent.EventType = curEvent;
                    thisEvent.EventParam = moveDirection;
                    #ifndef EVENTCHECKER_TEST           // keep this as is for test harness
                        PostHSM(thisEvent);
                    #else
                        SaveEvent(thisEvent);
                    #endif
                }
                
                break;
            case DiagTurnL:
                averageRear = (abs(RearRightState.count) + abs(RearLeftState.count)) >> 1;
                if (averageRear > moveAmount) {
                    moveActive = FALSE;
                    curEvent = FINISHED_MOVING;
                    thisEvent.EventType = curEvent;
                    thisEvent.EventParam = moveDirection;
                    returnVal = TRUE;
                    #ifndef EVENTCHECKER_TEST           // keep this as is for test harness
                        PostHSM(thisEvent);
                    #else
                        SaveEvent(thisEvent);
                    #endif
                }
                break;
                
            case DiagTurnR:
                averageFront = (abs(FrontLeftState.count) + abs(FrontRightState.count)) >> 1;
                if (averageFront > moveAmount) {
                    moveActive = FALSE;
                    curEvent = FINISHED_MOVING;
                    thisEvent.EventType = curEvent;
                    thisEvent.EventParam = moveDirection;
                    returnVal = TRUE;
                    #ifndef EVENTCHECKER_TEST           // keep this as is for test harness
                        PostHSM(thisEvent);
                    #else
                        SaveEvent(thisEvent);
                    #endif
                }
                break;
            case HalfTankL:
                averageFront = (abs(FrontLeftState.count) + abs(RearLeftState.count)) >> 1;
                if (averageFront > moveAmount) {
                    moveActive = FALSE;
                    curEvent = FINISHED_MOVING;
                    thisEvent.EventType = curEvent;
                    thisEvent.EventParam = moveDirection;
                    returnVal = TRUE;
                    #ifndef EVENTCHECKER_TEST           // keep this as is for test harness
                        PostHSM(thisEvent);
                    #else
                        SaveEvent(thisEvent);
                    #endif
                }
                break;
            case HalfTankR:
                averageFront = (abs(FrontRightState.count) + abs(RearRightState.count)) >> 1;
                if (averageFront > moveAmount) {
                    moveActive = FALSE;
                    curEvent = FINISHED_MOVING;
                    thisEvent.EventType = curEvent;
                    thisEvent.EventParam = moveDirection;
                    returnVal = TRUE;
                    #ifndef EVENTCHECKER_TEST           // keep this as is for test harness
                        PostHSM(thisEvent);
                    #else
                        SaveEvent(thisEvent);
                    #endif
                }
                break;
            default:
                break;


        }
    }
    return (returnVal);
}

//uint8_t PingChecker(void) {
//    static PingStates states = RHigh;
//
//    ES_EventTyp_t curEvent;
//    ES_Event thisEvent;
//    uint8_t returnVal = FALSE;
//
//    switch (states) {
//        case LHigh:
//            if (PORT_ECHO_L == 0) {
//                states = LLow;
//                lastLeftDistance = CUR_TIME - lastLeftTime;
//                lastLeftDistance /= 148;
//                curEvent = LEFT_ECHO;
//                thisEvent.EventType = curEvent;
//                thisEvent.EventParam = lastLeftDistance;
//                returnVal = TRUE;
//                PostPingSensorService(thisEvent);
//                printf("lH\r\n");
//            }
//            break;
//
//        case LLow:
//            if (PORT_ECHO_R == 1) {
//                states = RHigh;
//                lastRightTime = CUR_TIME;
//                printf("lL\r\n");
//            }
//
//            break;
//
//        case RHigh:
//            if (PORT_ECHO_R == 0) {
//                states = RLow;
//                lastRightDistance = CUR_TIME - lastRightTime;
//                lastRightDistance /= 148;
//                curEvent = RIGHT_ECHO;
//                thisEvent.EventType = curEvent;
//                thisEvent.EventParam = lastRightDistance;
//                returnVal = TRUE;
//                PostPingSensorService(thisEvent);
//                printf("rH\r\n");
//            }
//
//            break;
//
//        case RLow:
//            if (PORT_ECHO_L == 1) {
//                states = LHigh;
//                lastLeftTime = CUR_TIME;
//                printf("rL\r\n");
//            }
//
//            break;
//    }
//    return returnVal;
//}



/* 
 * The Test Harness for the event checkers is conditionally compiled using
 * the EVENTCHECKER_TEST macro (defined either in the file or at the project level).
 * No other main() can exist within the project.
 * 
 * It requires a valid ES_Configure.h file in the project with the correct events in 
 * the enum, and the correct list of event checkers in the EVENT_CHECK_LIST.
 * 
 * The test harness will run each of your event detectors identically to the way the
 * ES_Framework will call them, and if an event is detected it will print out the function
 * name, event, and event parameter. Use this to test your event checking code and
 * ensure that it is fully functional.
 * 
 * If you are locking up the output, most likely you are generating too many events.
 * Remember that events are detectable changes, not a state in itself.
 * 
 * Once you have fully tested your event checking code, you can leave it in its own
 * project and point to it from your other projects. If the EVENTCHECKER_TEST marco is
 * defined in the project, no changes are necessary for your event checkers to work
 * with your other projects.
 */
#ifdef EVENTCHECKER_TEST
#include <stdio.h>
static uint8_t(*EventList[])(void) = {EVENT_CHECK_LIST};

void PrintEvent(void);

void main(void) {
    BOARD_Init();
    /* user initialization code goes here */

    // Do not alter anything below this line
    int i;

    printf("\r\nEvent checking test harness for %s", __FILE__);

    while (1) {
        if (IsTransmitEmpty()) {
            for (i = 0; i< sizeof (EventList) >> 2; i++) {
                if (EventList[i]() == TRUE) {
                    PrintEvent();
                    break;
                }

            }
        }
    }
}

void PrintEvent(void) {
    printf("\r\nFunc: %s\tEvent: %s\tParam: 0x%X", eventName,
            EventNames[storedEvent.EventType], storedEvent.EventParam);
}
#endif

//HELPER FUNCTIONS=====================================================================
//int GetAllWheelAvg(void) {
//    int32_t FRWheel = abs(Motor_FRQuadCount(void));
//    int32_t FLWheel = abs(Motor_FLQuadCount(void));
//    int32_t RRWheel = abs(Motor_RRQuadCount(void));
//    int32_t RLWheel = abs(Motor_RLQuadCount(void));
//
//    int64_t sum = (FRWheel + FLWheel + RRWheel + RLWheel);
//    int avg = (sum / 4);
//    return avg;
//}
//
//int GetLeftBankAvg(void) {
//    int32_t FLWheel = abs(Motor_FLQuadCount(void));
//    int32_t RLWheel = abs(Motor_RLQuadCount(void));
//    int64_t sum = (FLWheel + RLWheel);
//    int avg = (sum/2);
//    return avg;
//}
//
//int GetRightBankAvg(void) {
//    int32_t FRWheel = abs(Motor_FRQuadCount(void));
//    int32_t RRWheel = abs(Motor_RRQuadCount(void));
//    int64_t sum = (FRWheel + RRWheel);
//    int avg = (sum/2);
//    return avg;
//}

Motors RunMotorSM(Motors MotorInput, uint16_t newAB) {
    Motors returnMotor;
    returnMotor.count = MotorInput.count;
    returnMotor.states = MotorInput.states;
    switch (returnMotor.states) {
        case State00:
            if (newAB == 0b10) {
                returnMotor.count++;
                returnMotor.states = State10;
            } else if (newAB == 0b01) {
                returnMotor.count--;
                returnMotor.states = State01;
            }
            break;

        case State10:
            if (newAB == 0b11) {
                returnMotor.count++;
                returnMotor.states = State11;
            } else if (newAB == 0b00) {
                returnMotor.count--;
                returnMotor.states = State00;
            }
            break;

        case State11:
            if (newAB == 0b01) {
                returnMotor.count++;
                returnMotor.states = State01;
            } else if (newAB == 0b10) {
                returnMotor.count--;
                returnMotor.states = State10;
            }
            break;

        case State01:
            if (newAB == 0b00) {
                returnMotor.count++;
                returnMotor.states = State00;
            } else if (newAB == 0b11) {
                returnMotor.count--;
                returnMotor.states = State11;
            }
            break;

    }
    return returnMotor;
}

int32_t Motor_FRQuadCount(void) {
    return FrontRightState.count;
}

int32_t Motor_FLQuadCount(void) {
    return -FrontLeftState.count;
}

int32_t Motor_RRQuadCount(void) {
    return RearRightState.count;
}

int32_t Motor_RLQuadCount(void) {
    return -RearLeftState.count;
}