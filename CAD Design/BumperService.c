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
#include "BumperService.h"
#include <stdio.h>
#include "xc.h"
#include "TopLvlHSM.h"
#include "IO_Ports.h"

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/

#define BATTERY_DISCONNECT_THRESHOLD 175
#define ARRAYSIZE 3
#define TRIS_SEL0 PORTY07_TRIS
#define TRIS_SEL1 PORTY09_TRIS
#define TRIS_INPUT PORTY06_TRIS
#define TRIS_TOWER_LEFT PORTY11_TRIS
#define TRIS_TOWER_RIGHT PORTY08_TRIS
#define SEL0 PORTY07_LAT
#define SEL1 PORTY09_LAT
#define INPUT PORTY06_BIT
#define TOWER_LEFT PORTY11_BIT
#define TOWER_RIGHT PORTY08_BIT

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES                                                 *
 ******************************************************************************/
/* Prototypes for private functions for this machine. They should be functions
   relevant to the behavior of this state machine */
void SetFront(void);
void SetRear(void);
void SetLeft(void);
void SetRight(void);
/*******************************************************************************
 * PRIVATE MODULE VARIABLES                                                    *
 ******************************************************************************/
/* You will need MyPriority and maybe a state variable; you may need others
 * as well. */

static uint8_t MyPriority;
static uint8_t TowerLeftArray[ARRAYSIZE];
static uint8_t TowerRightArray[ARRAYSIZE];
static uint8_t FrontArray[ARRAYSIZE];
static uint8_t RearArray[ARRAYSIZE];
static uint8_t LeftArray[ARRAYSIZE];
static uint8_t RightArray[ARRAYSIZE];
static uint8_t TowerLeftIndex;
static uint8_t TowerRightIndex;
static uint8_t FrontIndex;
static uint8_t RearIndex;
static uint8_t LeftIndex;
static uint8_t RightIndex;
static ES_EventTyp_t lastTowerLeftBumper = BUMPER_RELEASED;
static ES_EventTyp_t lastTowerRightBumper = BUMPER_RELEASED;
static ES_EventTyp_t lastFrontBumper = BUMPER_RELEASED;
static ES_EventTyp_t lastRearBumper = BUMPER_RELEASED;
static ES_EventTyp_t lastLeftBumper = BUMPER_RELEASED;
static ES_EventTyp_t lastRightBumper = BUMPER_RELEASED;


static BumperStates curBumper;

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
uint8_t InitBumperService(uint8_t Priority) {
    ES_Event ThisEvent;

    MyPriority = Priority;

    // in here you write your initialization code
    // this includes all hardware and software initialization
    // that needs to occur.
    FrontIndex = 0;
    RearIndex = 0;
    LeftIndex = 0;
    RightIndex = 0;
    TowerLeftIndex = 0;
    TowerRightIndex = 0;
    TRIS_INPUT = 1;
    TRIS_SEL0 = 0;
    TRIS_SEL1 = 0;
    TRIS_TOWER_LEFT = 1;
    TRIS_TOWER_RIGHT = 1;            
    curBumper = FrontBumper;
    SetFront();
    // post the initial transition event
    ThisEvent.EventType = ES_INIT;
    ES_Timer_InitTimer(BUMPER_TIMER, BUMPER_TICK);
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
uint8_t PostBumperService(ES_Event ThisEvent) {
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
ES_Event RunBumperService(ES_Event ThisEvent) {
    ES_Event ReturnEvent;
    ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

    /********************************************
     in here you write your service code
     *******************************************/
    ES_EventTyp_t curEvent;

    int i;
    int sum = 0;
    switch (ThisEvent.EventType) {
        case ES_INIT:
            // No hardware initialization or single time setups, those
            // go in the init function above.
            //
            // This section is used to reset service for some reason
            break;

        case ES_TIMEOUT:
            TowerLeftArray[TowerLeftIndex] = TOWER_LEFT;
            TowerLeftIndex = (TowerLeftIndex + 1) % ARRAYSIZE;
            for (i = 0; i < ARRAYSIZE; i++) {
                sum += TowerLeftArray[i];
            }
            if (sum == ARRAYSIZE) {
                curEvent = BUMPER_PRESSED;
            } else if (sum == 0) {
                curEvent = BUMPER_RELEASED;
            } else {
                curEvent = lastTowerLeftBumper;
            }
            if (curEvent != lastTowerLeftBumper) { // check for change from last time
                printf("Tower Left Bumper Event: %d\r\n", curEvent);
                ReturnEvent.EventType = curEvent;
                ReturnEvent.EventParam = TowerLeftBumper;
                lastTowerLeftBumper = curEvent; // update history
#ifndef SIMPLESERVICE_TEST           // keep this as is for test harness
                PostHSM(ReturnEvent);
#else
                PostTemplateService(ReturnEvent);
#endif   
            }
            sum = 0;
            
            TowerRightArray[TowerRightIndex] = TOWER_RIGHT;
            TowerRightIndex = (TowerRightIndex + 1) % ARRAYSIZE;
            for (i = 0; i < ARRAYSIZE; i++) {
                sum += TowerRightArray[i];
            }
            if (sum == ARRAYSIZE) {
                curEvent = BUMPER_PRESSED;
            } else if (sum == 0) {
                curEvent = BUMPER_RELEASED;
            } else {
                curEvent = lastTowerRightBumper;
            }
            if (curEvent != lastTowerRightBumper) { // check for change from last time
                ReturnEvent.EventType = curEvent;
                ReturnEvent.EventParam = TowerRightBumper;
                lastTowerRightBumper = curEvent; // update history
#ifndef SIMPLESERVICE_TEST           // keep this as is for test harness
                PostHSM(ReturnEvent);
#else
                PostTemplateService(ReturnEvent);
#endif   
            }
            sum = 0;
//            
            switch (curBumper) {
                case FrontBumper:
                    FrontArray[FrontIndex] = INPUT;
                    FrontIndex = (FrontIndex + 1) % ARRAYSIZE;
                    for (i = 0; i < ARRAYSIZE; i++) {
                        sum += FrontArray[i];
                    }
                    if (sum == ARRAYSIZE) {
                        curEvent = BUMPER_PRESSED;
                    } else if (sum == 0) {
                        curEvent = BUMPER_RELEASED;
                    } else {
                        curEvent = lastFrontBumper;
                    }
                    if (curEvent != lastFrontBumper) { // check for change from last time
                        printf("Front Bumper Event\r\n");
                        ReturnEvent.EventType = curEvent;
                        ReturnEvent.EventParam = FrontBumper;
                        lastFrontBumper = curEvent; // update history
#ifndef SIMPLESERVICE_TEST           // keep this as is for test harness
                        PostHSM(ReturnEvent);
#else
                        PostTemplateService(ReturnEvent);
#endif   
                    }
                    curBumper = RightBumper;
                    SetRight();
                    sum = 0;
                    break;

                case RightBumper:
                    RightArray[RightIndex] = INPUT;
                    RightIndex = (RightIndex + 1) % ARRAYSIZE;
                    for (i = 0; i < ARRAYSIZE; i++) {
                        sum += RightArray[i];
                    }
                    if (sum == ARRAYSIZE) {
                        curEvent = BUMPER_PRESSED;
                    } else if (sum == 0) {
                        curEvent = BUMPER_RELEASED;
                    } else {
                        curEvent = lastRightBumper;
                    }
                    if (curEvent != lastRightBumper) { // check for change from last time
                        printf("Right Bumper Event\r\n");
                        ReturnEvent.EventType = curEvent;
                        ReturnEvent.EventParam = RightBumper;
                        lastRightBumper = curEvent; // update history
#ifndef SIMPLESERVICE_TEST           // keep this as is for test harness
                        PostHSM(ReturnEvent);
#else
                        PostTemplateService(ReturnEvent);
#endif   
                    }
                    curBumper = RearBumper;
                    SetRear();
                    sum = 0;
                    break;

                case RearBumper:
                    RearArray[RearIndex] = INPUT;
                    RearIndex = (RearIndex + 1) % ARRAYSIZE;
                    for (i = 0; i < ARRAYSIZE; i++) {
                        sum += RearArray[i];
                    }
                    if (sum == ARRAYSIZE) {
                        curEvent = BUMPER_PRESSED;
                    } else if (sum == 0) {
                        curEvent = BUMPER_RELEASED;
                    } else {
                        curEvent = lastRearBumper;
                    }
                    if (curEvent != lastRearBumper) { // check for change from last time
                        printf("Rear Bumper Event\r\n");
                        ReturnEvent.EventType = curEvent;
                        ReturnEvent.EventParam = RearBumper;
                        lastRearBumper = curEvent; // update history
#ifndef SIMPLESERVICE_TEST           // keep this as is for test harness
                        PostHSM(ReturnEvent);
#else
                        PostTemplateService(ReturnEvent);
#endif   
                    }
                    curBumper = LeftBumper;
                    SetLeft();
                    sum = 0;
                    break;

                case LeftBumper:
                    LeftArray[LeftIndex] = INPUT;
                    LeftIndex = (LeftIndex + 1) % ARRAYSIZE;
                    for (i = 0; i < ARRAYSIZE; i++) {
                        sum += LeftArray[i];
                    }
                    if (sum == ARRAYSIZE) {
                        curEvent = BUMPER_PRESSED;
                    } else if (sum == 0) {
                        curEvent = BUMPER_RELEASED;
                    } else {
                        curEvent = lastLeftBumper;
                    }
                    if (curEvent != lastLeftBumper) { // check for change from last time
                        printf("Left Bumper Event\r\n");
                        ReturnEvent.EventType = curEvent;
                        ReturnEvent.EventParam = LeftBumper;
                        lastLeftBumper = curEvent; // update history
#ifndef SIMPLESERVICE_TEST           // keep this as is for test harness
                        PostHSM(ReturnEvent);
#else
                        PostTemplateService(ReturnEvent);
#endif   
                    }
                    curBumper = FrontBumper;
                    SetFront();
                    sum = 0;
                    break;

                default:
                    break;
            }
            ES_Timer_InitTimer(BUMPER_TIMER, BUMPER_TICK);

        default:
            break;
    }

    return ReturnEvent;
}

/*******************************************************************************
 * PRIVATE FUNCTIONs                                                           *
 ******************************************************************************/

void SetRear(void) {
    SEL0 = 0;
    SEL1 = 0;
}

void SetLeft(void) {
    SEL0 = 0;
    SEL1 = 1;
}

void SetFront(void) {
    SEL0 = 1;
    SEL1 = 1;
}

void SetRight(void) {
    SEL0 = 1;
    SEL1 = 0;
}

ES_EventTyp_t Bumper_LastTowerRight(void){
    return lastTowerRightBumper;
}
ES_EventTyp_t Bumper_LastTowerLeft(void){
    return lastTowerLeftBumper;
}
ES_EventTyp_t Bumper_LastRight(void){
    return lastRightBumper;
}
ES_EventTyp_t Bumper_LastFront(void){
    return lastFrontBumper;
}
ES_EventTyp_t Bumper_LastLeft(void){
    return lastLeftBumper;
}
ES_EventTyp_t Bumper_LastRear(void){
    return lastRearBumper;
}
