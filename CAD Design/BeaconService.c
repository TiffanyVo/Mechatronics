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
#include "BeaconService.h"
#include <stdio.h>
#include "TopLvlHSM.h"

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/

//needs events: BEACON_DETECTED, BEACON_NOT_DETECTED, BEACON_COMFIRMED, BEACON_UNCONFIRMED
//operation: to start sampling, startSampling must be called to begin the timer. movingSample
//will be called until stopSampling is called. A stop sample finishes after sample amount is reached

#define BEACON_TICK 100
#define BEACON_PIN AD_PORTW5
#define BEACON_LOW_THRESHOLD 100
#define BEACON_HIGH_THRESHOLD 200
#define SAMPLE_AMOUNT 10



static BeaconSampleMode mode = MovingSample;
static uint16_t samples[SAMPLE_AMOUNT];
static uint8_t sample_index;
static uint8_t isSampling = FALSE;
static uint32_t averageVal = 0;

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
uint8_t InitBeaconService(uint8_t Priority) {
    ES_Event ThisEvent;

    MyPriority = Priority;
    AD_AddPins(BEACON_PIN);

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
uint8_t PostBeaconService(ES_Event ThisEvent) {
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
ES_Event RunBeaconService(ES_Event ThisEvent) {
    ES_Event ReturnEvent;
    ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

    /********************************************
     in here you write your service code
     *******************************************/
    static ES_EventTyp_t lastEvent = BEACON_NOT_DETECTED;
    ES_EventTyp_t curEvent;
    uint16_t beaconVoltage = AD_ReadADPin(BEACON_PIN);
    int i;

    switch (ThisEvent.EventType) {
        case ES_INIT:
            break;

        case ES_TIMEOUT:
            switch (mode) {
                case MovingSample:
//                    printf("Beacon voltage: %d\r\n", beaconVoltage);
                    if (beaconVoltage > BEACON_HIGH_THRESHOLD) {
                        curEvent = BEACON_DETECTED;
                    } else if (beaconVoltage < BEACON_LOW_THRESHOLD) {
                        curEvent = BEACON_NOT_DETECTED;
                    } else {
                        curEvent = lastEvent;
                    }
                    if (curEvent != lastEvent) {
                        ReturnEvent.EventType = curEvent;
                        ReturnEvent.EventParam = beaconVoltage;
                        lastEvent = curEvent;
                        PostHSM(ReturnEvent);
                    }
                    break;

                case StoppedSample:
                    samples[sample_index] = beaconVoltage;
                    sample_index++;

                    if (sample_index == SAMPLE_AMOUNT) {
                        averageVal = 0;
                        for (i = 0; i < SAMPLE_AMOUNT; i++) {
                            averageVal += samples[i];
                        }
                        averageVal /= SAMPLE_AMOUNT;

                        if (averageVal > BEACON_HIGH_THRESHOLD) {
                            curEvent = BEACON_CONFIRMED;
                        } else {
                            curEvent = BEACON_UNCONFIRMED;
                        }

                        ReturnEvent.EventType = curEvent;
                        ReturnEvent.EventParam = averageVal;
                        PostHSM(ReturnEvent);

                        isSampling = FALSE;
                    }
                    break;

                case ContinuousSampling:
                    ReturnEvent.EventType = BEACON_SAMPLE;
                    ReturnEvent.EventParam = beaconVoltage;
                    lastEvent = curEvent;
                    PostHSM(ReturnEvent);
                    //printf("sample event: %d\r\n", beaconVoltage);
                    break;

            }
            if (isSampling) {
                ES_Timer_InitTimer(BEACON_TIMER, BEACON_TICK);
            }
            break;

#ifdef SIMPLESERVICE_TEST     // keep this as is for test harness      
        default:
            printf("\r\nEvent: %s\tParam: 0x%X",
                    EventNames[ThisEvent.EventType], ThisEvent.EventParam);
            break;
#endif
    }

    return ReturnEvent;
}

/*******************************************************************************
 * PRIVATE FUNCTIONs                                                           *
 ******************************************************************************/

void Beacon_SetMode(BeaconSampleMode m) {
    mode = m;
}

void Beacon_StartSampling() {
    isSampling = TRUE;
    int i;
    for (i = 0; i < SAMPLE_AMOUNT; i++) {
        samples[i] = 0;
    }
    sample_index = 0;
    ES_Timer_InitTimer(BEACON_TIMER, BEACON_TICK);
}

void Beacon_StopSampling() {
    isSampling = FALSE;
}
