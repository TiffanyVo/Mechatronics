/*
 * File: TemplateService.h
 * Author: J. Edward Carryer
 * Modified: Gabriel H Elkaim
 *
 * Template file to set up a simple service to work with the Events and Services
 * Framework (ES_Framework) on the Uno32 for the CMPE-118/L class. Note that this file
 * will need to be modified to fit your exact needs, and most of the names will have
 * to be changed to match your code.
 *
 * This is provided as an example and a good place to start.
 *
 * Created on 23/Oct/2011
 * Updated on 13/Nov/2013
 */

/*******************************************************************************
 * MODULE #INCLUDE                                                             *
 ******************************************************************************/

#include "BOARD.h"
#include "AD.h"
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "EncoderService.h"
#include <stdio.h>
#include <proc/p32mx320f128l.h>
#include <xc.h>
#include "MotorControl.h"

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/

#define BATTERY_DISCONNECT_THRESHOLD 175
#define TIMER_0_TICKS 100

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES                                                 *
 ******************************************************************************/
/* Prototypes for private functions for this machine. They should be functions
   relevant to the behavior of this state machine */

/*******************************************************************************
 * PRIVATE MODULE VARIABLES                                                    *
 ******************************************************************************/
/* You will need MyPriority and maybe a state variable; you may need others
 * as well. */

typedef enum MotorStates{
    State00, State10, State11, State01
} MotorStates;

typedef struct Motors{
    int32_t count;
    MotorStates states;
} Motors;

static uint8_t MyPriority;
static Motors FrontLeft;
static Motors FrontRight;
static Motors RearRight;
static Motors RearLeft;

Motors RunMotorSM(Motors MotorInput, uint16_t newAB);
int32_t Motor_FRQuadCount(void);
int32_t Motor_FLQuadCount(void);
int32_t Motor_RRQuadCount(void);
int32_t Motor_RLQuadCount(void);

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/

/**
 * @Function InitTemplateService(uint8_t Priority)
 * @param Priority - internal variable to track which event queue to use
 * @return TRUE or FALSE
 * @brief This will get called by the framework at the beginning of the code
 *        execution. It will post an ES_INIT event to the appropriate event
 *        queue, which will be handled inside RunTemplateService function. Remember
 *        to rename this to something appropriate.
 *        Returns TRUE if successful, FALSE otherwise
 * @author J. Edward Carryer, 2011.10.23 19:25 */
uint8_t InitEncoderService(uint8_t Priority) {
    ES_Event ThisEvent;

    MyPriority = Priority;

    // in here you write your initialization code
    // this includes all hardware and software initialization
    // that needs to occur.
    
    TRIS_FRONTLEFT_A = 1;  
    TRIS_FRONTLEFT_B = 1;  
    TRIS_FRONTRIGHT_A = 1;  
    TRIS_FRONTRIGHT_B = 1;  
    TRIS_REARLEFT_A = 1;   
    TRIS_REARLEFT_B = 1;  
    TRIS_REARRIGHT_A = 1; 
    TRIS_REARRIGHT_B = 1; 
    
    FrontLeft.states = State00;
    FrontLeft.count = 0;
    FrontRight.states = State00;
    FrontRight.count = 0;
    RearLeft.states = State00;
    RearLeft.count = 0;
    RearRight.states = State00;
    RearRight.count = 0;

    Motor_Init();
//    Motor_AllDutyCycle(800);
//    Motor_Reset();
//    Motor_Forward();
    // post the initial transition event
    ES_Timer_InitTimer(ENCODER_TIMER, TIMER_0_TICKS);
    ThisEvent.EventType = ES_INIT;
    if (ES_PostToService(MyPriority, ThisEvent) == TRUE) {
        return TRUE;
    } else {
        return FALSE;
    }
}

/**
 * @Function PostTemplateService(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be posted to queue
 * @return TRUE or FALSE
 * @brief This function is a wrapper to the queue posting function, and its name
 *        will be used inside ES_Configure to point to which queue events should
 *        be posted to. Remember to rename to something appropriate.
 *        Returns TRUE if successful, FALSE otherwise
 * @author J. Edward Carryer, 2011.10.23 19:25 */
uint8_t PostEncoderService(ES_Event ThisEvent) {
    return ES_PostToService(MyPriority, ThisEvent);
}

/**
 * @Function RunTemplateService(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be responded.
 * @return Event - return event (type and param), in general should be ES_NO_EVENT
 * @brief This function is where you implement the whole of the service,
 *        as this is called any time a new event is passed to the event queue. 
 * @note Remember to rename to something appropriate.
 *       Returns ES_NO_EVENT if the event have been "consumed." 
 * @author J. Edward Carryer, 2011.10.23 19:25 */
ES_Event RunEncoderService(ES_Event ThisEvent) {
    ES_Event ReturnEvent;
    ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

    /********************************************
     in here you write your service code
     *******************************************/
//    static ES_EventTyp_t lastEvent = BATTERY_DISCONNECTED;
//    ES_EventTyp_t curEvent;
//    uint16_t batVoltage = AD_ReadADPin(BAT_VOLTAGE); // read the battery voltage

    switch (ThisEvent.EventType) {
        case ES_INIT:
            // No hardware initialization or single time setups, those
            // go in the init function above.
            //
            // This section is used to reset service for some reason
            break;
        case ES_TIMERACTIVE: /* signals that a timer has become active */
        case ES_TIMERSTOPPED:
            break;
            
        case FRONT_LEFT_MOTOR:
            FrontLeft = RunMotorSM(FrontLeft, ThisEvent.EventParam);
//            printf("received FRONT LEFT\r\n");
//            if(Motor_FRQuadCount() > 2000){
//                Motor_Reset();
//            }
            break;
            
        case FRONT_RIGHT_MOTOR:
            FrontRight = RunMotorSM(FrontRight, ThisEvent.EventParam);
//            printf("received FRONT RIGHT\r\n");
            break;
            
        case REAR_LEFT_MOTOR:
            RearLeft = RunMotorSM(RearLeft, ThisEvent.EventParam);
//            printf("received REAR LEFT\r\n");
            break;
            
        case REAR_RIGHT_MOTOR:
            RearRight = RunMotorSM(RearRight, ThisEvent.EventParam);
//            printf("received REAR RIGHT\r\n");
            break;
            
        case ES_TIMEOUT:
            ES_Timer_InitTimer(ENCODER_TIMER, TIMER_0_TICKS);
//            printf("Current States:\r\nFRONT LEFT: %d,FRONT RIGHT: %d,REAR LEFT: %d,REAR RIGHT: %d\r\n", 
//                    Motor_FLQuadCount(), Motor_FRQuadCount(), Motor_RLQuadCount(), Motor_RRQuadCount());
//            if (batVoltage > BATTERY_DISCONNECT_THRESHOLD) { // is battery connected?
//                curEvent = BATTERY_CONNECTED;
//            } else {
//                curEvent = BATTERY_DISCONNECTED;
//            }
//            if (curEvent != lastEvent) { // check for change from last time
//                ReturnEvent.EventType = curEvent;
//                ReturnEvent.EventParam = batVoltage;
//                lastEvent = curEvent; // update history
//            }
//#ifndef SIMPLESERVICE_TEST           // keep this as is for test harness
//            PostGenericService(ReturnEvent);
//#else
//            PostTemplateService(ReturnEvent);
//#endif   
//            break;
//
//#ifdef SIMPLESERVICE_TEST     // keep this as is for test harness      
//        default:
//            printf("\r\nEvent: %s\tParam: 0x%X",
//                    EventNames[ThisEvent.EventType], ThisEvent.EventParam);
//            break;
//#endif
            break;
        default:
            break;
    }

    return ReturnEvent;
}

/*******************************************************************************
 * PRIVATE FUNCTIONs                                                           *
 ******************************************************************************/

//Motors RunMotorSM(Motors MotorInput, uint16_t newAB){
//    Motors returnMotor;
//    returnMotor.count = MotorInput.count;
//    returnMotor.states = MotorInput.states;
//    switch (returnMotor.states){
//        case State00:
//            if(newAB == 0b10){
//                returnMotor.count++;
//                returnMotor.states = State10;
//            }
//            else if(newAB == 0b01){
//                returnMotor.count--;
//                returnMotor.states = State01;
//            }
//            break;
//            
//        case State10:
//            if(newAB == 0b11){
//                returnMotor.count++;
//                returnMotor.states = State11;
//            }
//            else if(newAB == 0b00){
//                returnMotor.count--;
//                returnMotor.states = State00;
//            }
//            break;
//            
//        case State11:
//            if(newAB == 0b01){
//                returnMotor.count++;
//                returnMotor.states = State01;
//            }
//            else if(newAB == 0b10){
//                returnMotor.count--;
//                returnMotor.states = State10;
//            }
//            break;
//            
//        case State01:
//            if(newAB == 0b00){
//                returnMotor.count++;
//                returnMotor.states = State00;
//            }
//            else if(newAB == 0b11){
//                returnMotor.count--;
//                returnMotor.states = State11;
//            }
//            break;
//            
//    }
//    return returnMotor;
//}

