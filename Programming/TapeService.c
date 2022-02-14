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
#include "TapeService.h"
#include "IO_Ports.h"
#include <stdio.h>
#include <xc.h>

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/

static uint16_t tapeLevelsLight[8];
static uint16_t tapeLevelsDark[8];
static uint16_t tapeLevels[8];

static uint16_t tapeLowThreshold[8];
static uint16_t tapeHighThreshold[8];
static uint8_t lightFlag;
static ES_EventTyp_t lastEvents[6];
static ES_EventTyp_t lastTowerEvent;
static uint8_t inEdgeFollowing;



//#define TRIS_TAPE_1 PORTY03_TRIS
//#define LAT_TAPE_1 PORTY03_LAT
//#define TRIS_TAPE_2 PORTY09_TRIS
//#define LAT_TAPE_2 PORTY09_LAT
//#define TRIS_TAPE_3 PORTY11_TRIS
//#define LAT_TAPE_3 PORTY11_LAT

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

static uint8_t MyPriority;


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
uint8_t InitTapeService(uint8_t Priority) {

    ES_Event ThisEvent;

    MyPriority = Priority;

    // in here you write your initialization code
    // this includes all hardware and software initialization
    // that needs to occur.
    AD_Init();
    AD_AddPins(TAPE_L | TAPE_R | TAPE_FR | TAPE_FL | TAPE_RR | TAPE_RL | TAPE_TL | TAPE_TR);
    ES_Timer_InitTimer(TAPE_TIMER, TAPE_TICK);

    LED_TRIS = 0;
    LED_LAT = 1;
    inEdgeFollowing = 0;
    lightFlag = 0;
    //    TRIS_TAPE_1 = 0;
    //    TRIS_TAPE_2 = 0;
    //    TRIS_TAPE_3 = 0;
    //    LAT_TAPE_1 = 1;
    //    LAT_TAPE_2 = 1;
    //    LAT_TAPE_3 = 1;
    int i = 0;
    for (i = 0; i < 8; i++) {
        tapeLevels[i] = 0;
        //111 threshold
        //        tapeLowThreshold[i] = 200;
        //        tapeHighThreshold[i] = 500;

        //fab lab threshold
        tapeLowThreshold[i] = 200;
        tapeHighThreshold[i] = 450;
        lastEvents[i] = TAPE_OFF;
    }
    
    lastTowerEvent = OFF_TOWER;
    
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
uint8_t PostTapeService(ES_Event ThisEvent) {
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



ES_Event RunTapeService(ES_Event ThisEvent) {
    ES_Event ReturnEvent;
    ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

    /********************************************
     in here you write your service code
     *******************************************/
    int i;
    ES_EventTyp_t curEvent = ES_NO_EVENT;
    ES_Event thisEvent;

    switch (ThisEvent.EventType) {
        case ES_INIT:
            // No hardware initialization or single time setups, those
            // go in the init function above.
            //
            // This section is used to reset service for some reason
            break;

        case ES_TIMEOUT:
            tapeLevels[0] = AD_ReadADPin(TAPE_FR);
            tapeLevels[1] = AD_ReadADPin(TAPE_FL);
            tapeLevels[2] = AD_ReadADPin(TAPE_RR);
            tapeLevels[3] = AD_ReadADPin(TAPE_RL);
            tapeLevels[4] = AD_ReadADPin(TAPE_R);
            tapeLevels[5] = AD_ReadADPin(TAPE_L);
            tapeLevels[6] = AD_ReadADPin(TAPE_TL);
            tapeLevels[7] = AD_ReadADPin(TAPE_TR);

            //printf("Value for Front Left: %d, Front Right: %d\r\n",tapeLevels[1],  tapeLevels[0] );

            for (i = 0; i < 6; i++) {
                //printf("posted %d, value: %d \r\n", i, tapeLevels[i]);
                if (tapeLevels[i] > tapeHighThreshold[i]) {
                    curEvent = TAPE_ON;
                } else if (tapeLevels[i] < tapeLowThreshold[i]) {
                    curEvent = TAPE_OFF;
                } else {
                    curEvent = lastEvents[i];
                }
                if (curEvent != ES_NO_EVENT && (curEvent != lastEvents[i])) {
                    if (inEdgeFollowing) {
                        if (i == 0 || i == 1 || i == 4 || i == 5) {
                            thisEvent.EventType = curEvent;
                            thisEvent.EventParam = i;
                            lastEvents[i] = curEvent;
                            PostHSM(thisEvent); //PostEdgeFollowingFSM(thisEvent);
//                            printf("inEdgeFollowing: TapeEvent- Tape: %d, Event: %d\r\n", thisEvent.EventParam, thisEvent.EventType);
                        }
                    } else {
                        thisEvent.EventType = curEvent;
                        thisEvent.EventParam = i;
                        lastEvents[i] = curEvent;
                        PostHSM(thisEvent); //PostEdgeFollowingFSM(thisEvent);
                        if(i == RR_INDEX){
//                            printf("TapeEvent- Tape: %d, Event: %d\r\n", thisEvent.EventParam, thisEvent.EventType);
                        }
                        
                    }
                }
            }
//            printf("left: %d, right: %d\r\n",tapeLevels[TL_INDEX],tapeLevels[TR_INDEX] );
            if(tapeLevels[TL_INDEX] < 600 || tapeLevels[TR_INDEX] < 600){
                curEvent = TOWER_OFF_TAPE;
            }
            else if(tapeLevels[TL_INDEX] > 900 && tapeLevels[TR_INDEX] > 900){
                curEvent = TOWER_ON_TAPE;
            }
            else{
                curEvent = lastTowerEvent;
            }
            
            if (curEvent != lastTowerEvent) {
                        thisEvent.EventType = curEvent;
//                        thisEvent.EventParam = tapeLevels[TL_INDEX];
                        lastTowerEvent = curEvent;
                        PostHSM(thisEvent); //PostEdgeFollowingFSM(thisEvent); 
//                        printf("TapeEvent- Tape: %d, Event: %d\r\n", thisEvent.EventParam, thisEvent.EventType);
                }
            
            ES_Timer_InitTimer(TAPE_TIMER, TAPE_TICK);
            break;
        default:
            break;
    }


    return ReturnEvent;
}

/*******************************************************************************
 * PRIVATE FUNCTIONs                                                           *
 ******************************************************************************/

void setEdgeFollowing(uint8_t value) {
    inEdgeFollowing = value;
}

ES_EventTyp_t CurrentTape(uint8_t index) {
    return lastEvents[index];
}

ES_EventTyp_t CurrentTowerTape(void) {
    return lastTowerEvent;
}
