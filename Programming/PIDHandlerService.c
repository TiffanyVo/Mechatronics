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
#include "PIDHandlerService.h"
#include "EncoderService.h"
#include "MotorControl.h"
#include <stdio.h>
#include "math.h"
#include "serial.h"
/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/
#define MIN(a,b) ((((int32_t)a)<((int32_t)b))?((int32_t)a):((int32_t)b))
#define MAX(a,b) ((((int32_t)a)>((int32_t)b))?((int32_t)a):((int32_t)b))

#define BATTERY_DISCONNECT_THRESHOLD 175
#define MAX_SPEED 3200
#define MIN_SPEED -3200
#define P_GAIN 1.5
#define I_GAIN 1
#define DELTA_T 0.005
#define MAX_RESPONSE 1000
#define MIN_RESPONSE -1000
#define TIMER_0_TICKS 100
#define FEEDBACK_MAXOUTPUT_POWER 27
#define MAX_CONTROL_OUTPUT (1<<FEEDBACK_MAXOUTPUT_POWER)


/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES                                                 *
 ******************************************************************************/
/* Prototypes for private functions for this machine. They should be functions
   relevant to the behavior of this state machine */
uint16_t PID(int32_t actualSpeed);
int FeedbackControl_Update(int referenceValue, int sensorValue);
/*******************************************************************************
 * PRIVATE MODULE VARIABLES                                                    *
 ******************************************************************************/
/* You will need MyPriority and maybe a state variable; you may need others
 * as well. */

static uint8_t MyPriority;
static int16_t refSpeed;
static int16_t prevSpeed;
static int32_t FRcurSpeed;
static int32_t FLcurSpeed;
static int32_t RRcurSpeed;
static int32_t RLcurSpeed;
static int32_t refTicks;
static int32_t integrator;
static int32_t FRLastTick;
static int32_t FLLastTick;
static int32_t RRLastTick;
static int32_t RLLastTick;
static int32_t FRCurTick;
static int32_t FLCurTick;
static int32_t RRCurTick;
static int32_t RLCurTick;

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
uint8_t InitPIDHandlerService(uint8_t Priority)
{
    ES_Event ThisEvent;

    MyPriority = Priority;

    // in here you write your initialization code
    // this includes all hardware and software initialization
    // that needs to occur.
    refSpeed = 3200;
    prevSpeed = 0;
    refTicks=0;
    integrator=0;
    FRLastTick=0;
    FLLastTick=0;
    RRLastTick=0;
    RLLastTick=0;
    FRCurTick=0;
    FLCurTick=0;
    RRCurTick=0;
    RLCurTick=0;
    ES_Timer_InitTimer(ENCODER_TIMER, TIMER_0_TICKS);
    Motor_Init();
    // post the initial transition event
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
uint8_t PostPIDHandlerService(ES_Event ThisEvent)
{
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
ES_Event RunPIDHandlerService(ES_Event ThisEvent)
{
    ES_Event ReturnEvent;
    ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

    /********************************************
     in here you write your service code
     *******************************************/
    static ES_EventTyp_t lastEvent = BATTERY_DISCONNECTED;
    ES_EventTyp_t curEvent;
    uint16_t batVoltage = AD_ReadADPin(BAT_VOLTAGE); // read the battery voltage

    switch (ThisEvent.EventType) {
    case ES_INIT:
        // No hardware initialization or single time setups, those
        // go in the init function above.
        //
        // This section is used to reset service for some reason
        break;

    case ES_TIMEOUT:
        if(ThisEvent.EventParam == ENCODER_TIMER){
            
            FRCurTick = Motor_FRQuadCount();
            FLCurTick = Motor_FLQuadCount();
            RRCurTick = Motor_RRQuadCount();
            RLCurTick = Motor_RLQuadCount();
            FRcurSpeed = (FRCurTick - FRLastTick)*10;
            FLcurSpeed = (FLCurTick - FLLastTick)*10;
            RRcurSpeed = (RRCurTick - RRLastTick)*10;
            RLcurSpeed = (RLCurTick - RLLastTick)*10;
//            printf("curTick: %d, prevTick: %d, current speed: %d\r\n",FRCurTick, FRLastTick, curSpeed);
//            printf("%d\r\n",refSpeed - curSpeed );
            long long int FRu = FeedbackControl_Update(refSpeed, FRcurSpeed);
            long long int FLu = FeedbackControl_Update(refSpeed, FLcurSpeed);
            long long int RRu = FeedbackControl_Update(refSpeed, RRcurSpeed);
            long long int RLu = FeedbackControl_Update(refSpeed, RLcurSpeed);
            
            FRu = ((FRu * 1000)/ MAX_CONTROL_OUTPUT);
            if(FRu >= 0){
                RIGHT_FRONT_FORWARD =  1;
                RIGHT_FRONT_REVERSE = 0;
            }
            FLu = ((FLu * 1000)/ MAX_CONTROL_OUTPUT);
            if(FLu >= 0){
                LEFT_FRONT_FORWARD =  1;
                LEFT_FRONT_REVERSE = 0;
            }
            RRu = ((RRu * 1000)/ MAX_CONTROL_OUTPUT);
            if(RRu >= 0){
                RIGHT_REAR_FORWARD =  1;
                RIGHT_REAR_REVERSE = 0;
            }
            RLu = ((RLu * 1000)/ MAX_CONTROL_OUTPUT);
            if(RLu >= 0){
                LEFT_REAR_FORWARD =  1;
                LEFT_REAR_REVERSE = 0;
            }
            
            PWM_SetDutyCycle(RIGHT_FRONT_ENABLE, 800); // FRu);
            PWM_SetDutyCycle(RIGHT_REAR_ENABLE, 800);  //RRu);
            PWM_SetDutyCycle(LEFT_FRONT_ENABLE, 800); //FLu);
            PWM_SetDutyCycle(LEFT_REAR_ENABLE, 800);  //RLu);
            
//            
//            FRCurTick = Motor_FRQuadCount();
//            FLCurTick = Motor_FLQuadCount();
//            RRCurTick = Motor_RRQuadCount();
//            RLCurTick = Motor_RLQuadCount();
//            int16_t FRResponse = PID();
//            int16_t FLResponse = PID((FLCurTick - FLLastTick)*200);
//            int16_t RRResponse = PID((RRCurTick - RRLastTick)*200);
//            int16_t RLResponse = PID((RLCurTick - RLLastTick)*200);
//            RIGHT_FRONT_FORWARD = (FRResponse >= 0);
//            RIGHT_FRONT_REVERSE = !(FRResponse >= 0);
//            PWM_SetDutyCycle(RIGHT_FRONT_ENABLE, (FRResponse < 0) ? -FRResponse : FRResponse);
//            PWM_SetDutyCycle(RIGHT_REAR_ENABLE, (RRResponse < 0) ? -RRResponse : RRResponse);
//            PWM_SetDutyCycle(LEFT_FRONT_ENABLE, (FLResponse < 0) ? FLResponse : -FLResponse);
//            PWM_SetDutyCycle(LEFT_REAR_ENABLE, (RLResponse < 0) ? RLResponse : -RLResponse);
//            printf("Front Left response: %d, current speed: %d\r\n",FRResponse,(FRCurTick - FRLastTick)*200 );
            
            ES_Timer_InitTimer(ENCODER_TIMER, TIMER_0_TICKS);
            FRLastTick = FRCurTick;
            FLLastTick = FLCurTick;
            RRLastTick = RRCurTick;
            RLLastTick = RLCurTick;
        }
//        if (batVoltage > BATTERY_DISCONNECT_THRESHOLD) { // is battery connected?
//            curEvent = BATTERY_CONNECTED;
//        } else {
//            curEvent = BATTERY_DISCONNECTED;
//        }
//        if (curEvent != lastEvent) { // check for change from last time
//            ReturnEvent.EventType = curEvent;
//            ReturnEvent.EventParam = batVoltage;
//            lastEvent = curEvent; // update history
//#ifndef SIMPLESERVICE_TEST           // keep this as is for test harness
//            PostGenericService(ReturnEvent);
//#else
//            PostTemplateService(ReturnEvent);
//#endif   
//        }
//        break;
//#ifdef SIMPLESERVICE_TEST     // keep this as is for test harness      
//    default:
//        printf("\r\nEvent: %s\tParam: 0x%X",
//                EventNames[ThisEvent.EventType], ThisEvent.EventParam);
//        break;
//#endif
    }

    return ReturnEvent;
}

/*******************************************************************************
 * PRIVATE FUNCTIONs                                                           *
 ******************************************************************************/

//uint16_t PID(int32_t actualSpeed){
//    uint32_t error = refSpeed - actualSpeed;
//    integrator = integrator + error*DELTA_T;
//    float response = P_GAIN*error + I_GAIN*integrator;
//    return (uint32_t)MAX(-1000, MIN(1000, response));             // Clamp
//}



static int KP = 30000;
static int KI = 5000;
static int KD = 1000;

static int A = 0; //integral accumulator
static int previousSensorValue = 0;

int FeedbackControl_Update(int referenceValue, int sensorValue){
    int error = referenceValue - sensorValue; //calculate error
    A += error; //integrate error
    int D = -1*(sensorValue - previousSensorValue); //numerical derivative
    previousSensorValue = sensorValue; //update previous sensor value
    
    int u = (KP*error)+(KI*A)+(KD*D); //compute control

    if(u > MAX_CONTROL_OUTPUT){
        u = MAX_CONTROL_OUTPUT; //clip control output
        A -= error; //anti-windup
        return u;
    }
    if(u < (-1*MAX_CONTROL_OUTPUT)){
        u = (-1*MAX_CONTROL_OUTPUT);//clip control output
        A -= error; //anti-windup
        return u;
    }
    
    return u;
}