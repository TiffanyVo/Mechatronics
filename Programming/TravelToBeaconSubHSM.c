/*
 * File: TemplateSubHSM.c
 * Author: J. Edward Carryer
 * Modified: Gabriel H Elkaim
 *
 * Template file to set up a Heirarchical State Machine to work with the Events and
 * Services Framework (ES_Framework) on the Uno32 for the CMPE-118/L class. Note that
 * this file will need to be modified to fit your exact needs, and most of the names
 * will have to be changed to match your code.
 *
 * There is for a substate machine. Make sure it has a unique name
 *
 * This is provided as an example and a good place to start.
 *
 * History
 * When           Who     What/Why
 * -------------- ---     --------
 * 09/13/13 15:17 ghe      added tattletail functionality and recursive calls
 * 01/15/12 11:12 jec      revisions for Gen2 framework
 * 11/07/11 11:26 jec      made the queue static
 * 10/30/11 17:59 jec      fixed references to CurrentEvent in RunTemplateSM()
 * 10/23/11 18:20 jec      began conversion from SMTemplate.c (02/20/07 rev)
 */


/*******************************************************************************
 * MODULE #INCLUDE                                                             *
 ******************************************************************************/

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "BOARD.h"
#include "TopLvlHSM.h"
#include "ParallelVatSubHSM.h"
#include "TravelToBeaconSubHSM.h"
#include "BeaconService.h"
#include "MotorControl.h"
#include "TapeService.h"
#include "EventCheckers.h"
#include "EdgeFollowingFSM.h"
#include "EdgeFollowingSubHSM.h"

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/
typedef enum {
    StopAndSample,
    StrafeLeft,
    StrafeRight,
    ForwardZ,
    ForwardY,
    Reverse2Y,
    ForwardToMax
} TravelToBeaconSubHSMState_t;

static const char *StateNames[] = {
    "StopAndSample",
    "StrafeLeft",
    "StrafeRight",
    "ForwardZ",
    "ForwardY",
    "Reverse2Y",
    "ForwardToMax"
};

typedef struct {
    uint16_t beaconValue;
    int32_t encoderValue;
} BeaconSample;

#define FIND_AMOUNT 4
#define SAMPLE_AMOUNT 100
#define STRAFE_RIGHT_AMOUNT 6


/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES                                                 *
 ******************************************************************************/
/* Prototypes for private functions for this machine. They should be functions
   relevant to the behavior of this state machine */
BeaconSample findMaxBeacon();
BeaconSample findMaxDistance();
/*******************************************************************************
 * PRIVATE MODULE VARIABLES                                                            *
 ******************************************************************************/
/* You will need MyPriority and the state variable; you may need others as well.
 * The type of state variable should match that of enum in header file. */

static TravelToBeaconSubHSMState_t CurrentState = StopAndSample; // <- change name to match ENUM
static uint8_t MyPriority;

static BeaconSample beaconSamples[SAMPLE_AMOUNT];
static uint16_t samplesTaken = 0;
static BeaconSample maxSample;
static BeaconSample maxDistanceSample;
/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/

/**
 * @Function InitTemplateSubHSM(uint8_t Priority)
 * @param Priority - internal variable to track which event queue to use
 * @return TRUE or FALSE
 * @brief This will get called by the framework at the beginning of the code
 *        execution. It will post an ES_INIT event to the appropriate event
 *        queue, which will be handled inside RunTemplateFSM function. Remember
 *        to rename this to something appropriate.
 *        Returns TRUE if successful, FALSE otherwise
 * @author J. Edward Carryer, 2011.10.23 19:25 */
uint8_t InitTravelToBeaconSubHSM(void) {
    ES_Event returnEvent;

    CurrentState = StopAndSample;
    returnEvent = RunTravelToBeaconSubHSM(INIT_EVENT);
    if (returnEvent.EventType == ES_NO_EVENT) {
        return TRUE;
    }
    return FALSE;
}

/**
 * @Function RunTemplateSubHSM(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be responded.
 * @return Event - return event (type and param), in general should be ES_NO_EVENT
 * @brief This function is where you implement the whole of the heirarchical state
 *        machine, as this is called any time a new event is passed to the event
 *        queue. This function will be called recursively to implement the correct
 *        order for a state transition to be: exit current state -> enter next state
 *        using the ES_EXIT and ES_ENTRY events.
 * @note Remember to rename to something appropriate.
 *       The lower level state machines are run first, to see if the event is dealt
 *       with there rather than at the current level. ES_EXIT and ES_ENTRY events are
 *       not consumed as these need to pass pack to the higher level state machine.
 * @author J. Edward Carryer, 2011.10.23 19:25
 * @author Gabriel H Elkaim, 2011.10.23 19:25 */
ES_Event RunTravelToBeaconSubHSM(ES_Event ThisEvent) {
    uint8_t makeTransition = FALSE; // use to flag transition
    TravelToBeaconSubHSMState_t nextState; // <- change type to correct enum


    //ES_Tattle(); // trace call stack
    int i = 0;

    switch (CurrentState) {
        case StopAndSample: //STAYS IN HERE FOR 10-ISH SAMPLES
            switch (ThisEvent.EventType) {
                    printf("got to this thing\r\n");
                case ES_INIT:
                    break;
                case ES_ENTRY:
                    Beacon_StopSampling();
                    Beacon_SetMode(StoppedSample);
                    Beacon_StartSampling();
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;
                case BEACON_CONFIRMED:
                    //                    setFinishedTower(1);
                    nextState = StrafeRight;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;
                case BEACON_UNCONFIRMED:
                    nextState = StopAndSample; //THIS WOULD GO TO EDGE FOLLOWING
                    makeTransition = TRUE;
                    ThisEvent.EventType = GO_TO_EDGEFOLLOWING;
                    ThisEvent.EventParam = 1;
                    break;
                case ES_EXIT:
                    Motor_Reset();
                    Motor_AllDutyCycle(0);
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;
                default:
                    break;
            }
            break;

        case StrafeRight: // in the first state, replace this with correct names
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    // Set Bot to Strafe Right
                    SetMoveAmount(Move, InchesToTicks(STRAFE_RIGHT_AMOUNT));
                    Motor_StrafeRight();
                    Motor_AllDutyCycle(700);

                    // Take Moving Sample (from stopped sample in previous state)
                    Beacon_StopSampling();
                    Beacon_SetMode(MovingSample);
                    Beacon_StartSampling();

                    ThisEvent.EventType = ES_NO_EVENT;
                    break;
                case ES_NO_EVENT:
                    break;

                case BUMPER_PRESSED:
                    nextState = StopAndSample; //SHOULD GO TO CHECK IF TOWER SUBSTATE
                    makeTransition = TRUE;
                    ThisEvent.EventType = CHECK_IF_TOWER;
                    break;

                    //                case BEACON_NOT_DETECTED: //beacon lost
                    //                    nextState = StrafeLeft;
                    //                    makeTransition = TRUE;
                    //                    ThisEvent.EventType = ES_NO_EVENT;
                    //                    break;

                case FINISHED_MOVING:
                    nextState = ForwardY;
                    Motor_Reset();
                    Motor_AllDutyCycle(0);

                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case ES_EXIT:
                    Motor_Reset();
                    Motor_AllDutyCycle(0);
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;
                default: // all unhandled events pass the event back up to the next level
                    break;
            }
            break;

        case ForwardY:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    Motor_Reverse();
                    Motor_AllDutyCycle(700);

                    SetMoveAmount(Move, InchesToTicks(FIND_AMOUNT));
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case FINISHED_MOVING:
                    StopMovement();
                    nextState = Reverse2Y;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                    //                case BEACON_DETECTED:
                    //                    nextState = StopAndSample;
                    //                    makeTransition = TRUE;
                    //                    ThisEvent.EventType = ES_NO_EVENT;
                    //                    break;

                    //                case BUMPER_PRESSED:
                    //                    nextState = StopAndSample; //SHOULD GO TO CHECK IF TOWER SUBSTATE
                    //                    makeTransition = TRUE;
                    //                    ThisEvent.EventType = CHECK_IF_TOWER;
                    break;

                case ES_EXIT:
                    Motor_Reset();
                    Motor_AllDutyCycle(0);
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;
            }
            break;

        case Reverse2Y:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    Motor_Forward();
                    Motor_AllDutyCycle(700);

                    Beacon_StopSampling();
                    samplesTaken = 0;
                    Beacon_SetMode(ContinuousSampling);
                    Beacon_StartSampling();

                    SetMoveAmount(Move, InchesToTicks(FIND_AMOUNT * 2));
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case FINISHED_MOVING:
                    StopMovement();
                    Beacon_StopSampling();
                    maxSample = findMaxBeacon();
                    maxDistanceSample = findMaxDistance();
                    nextState = ForwardToMax;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case BEACON_DETECTED:
                    nextState = StopAndSample;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                    //                case BUMPER_PRESSED:
                    //                    nextState = CHECK_IF_TOWER; //SHOULD GO TO CHECK IF TOWER SUBSTATE
                    //                    makeTransition = TRUE;
                    //                    ThisEvent.EventType = CHECK_IF_TOWER;
                    //                    break;

                case ES_EXIT:
                    Motor_Reset();
                    Motor_AllDutyCycle(0);
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case BEACON_SAMPLE:
                    if (samplesTaken < SAMPLE_AMOUNT) {
                        beaconSamples[samplesTaken].beaconValue = ThisEvent.EventParam;
                        beaconSamples[samplesTaken].encoderValue = abs(Motor_FRQuadCount());
                        samplesTaken++;
                    }
                    break;
                case TAPE_ON:
                    if (ThisEvent.EventParam == FR_INDEX || ThisEvent.EventParam == FL_INDEX) {
                        StopMovement();
                        Beacon_StopSampling();
                        maxSample = findMaxBeacon();
                        maxDistanceSample = findMaxDistance();
                        nextState = ForwardToMax;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    break;
            }
            break;

        case StrafeLeft:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    // Set Bot to Strafe Left
                    Motor_StrafeLeft();
                    Motor_AllDutyCycle(700);
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case BEACON_DETECTED:
                    nextState = StopAndSample;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;
                case TAPE_ON:
                    if (ThisEvent.EventParam == FR_INDEX) { //this would be front and rear tape on
                        nextState = StopAndSample;
                        makeTransition = TRUE;
                        ThisEvent.EventType = GO_TO_EDGEFOLLOWING;
                        break;
                    }
                case ES_EXIT:
                    Motor_Reset();
                    Motor_AllDutyCycle(0);
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;
            }
            break;

        case ForwardToMax:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    Motor_Reverse();
                    Motor_AllDutyCycle(700);
                    SetMoveAmount(Move, (maxDistanceSample.encoderValue - maxSample.encoderValue) - InchesToTicks(3));
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case FINISHED_MOVING:
                    StopMovement();
                    Motor_AllDutyCycle(700);

                    nextState = StrafeRight;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                    //                case BUMPER_PRESSED:
                    //                    nextState = CHECK_IF_TOWER; //SHOULD GO TO CHECK IF TOWER SUBSTATE
                    //                    makeTransition = TRUE;
                    //                    ThisEvent.EventType = CHECK_IF_TOWER;
                    //                    break;

                case ES_EXIT:
                    Motor_Reset();
                    Motor_AllDutyCycle(0);
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

            }
            break;
        default: // all unhandled states fall into here
            break;
    } // end switch on Current State

    if (makeTransition == TRUE) { // making a state transition, send EXIT and ENTRY
        // recursively call the current state with an exit event
        RunTravelToBeaconSubHSM(EXIT_EVENT); // <- rename to your own Run function
        CurrentState = nextState;
        RunTravelToBeaconSubHSM(ENTRY_EVENT); // <- rename to your own Run function
    }

    ES_Tail(); // trace call stack end
    return ThisEvent;
}

/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/
BeaconSample findMaxBeacon() {
    int i;
    BeaconSample maxSample = beaconSamples[0];
    for (i = 1; i < samplesTaken; i++) {
        if (maxSample.beaconValue < beaconSamples[i].beaconValue) {
            maxSample = beaconSamples[i];
        }
    }
    return maxSample;
}

BeaconSample findMaxDistance() {
    int i;
    BeaconSample maxSample = beaconSamples[0];
    for (i = 1; i < samplesTaken; i++) {
        if (maxSample.encoderValue < beaconSamples[i].encoderValue) {
            maxSample = beaconSamples[i];
        }
    }
    return maxSample;
}