/*
 * File: TemplateSubHSM.c
 * Author: J. Edward Carryer
 * Modified: Gabriel Elkaim and Soja-Marie Morgens
 *
 * Template file to set up a Heirarchical State Machine to work with the Events and
 * Services Framework (ES_Framework) on the Uno32 for the CMPE-118/L class. Note that
 * this file will need to be modified to fit your exact needs, and most of the names
 * will have to be changed to match your code.
 *
 * There is another template file for the SubHSM's that is slightly differet, and
 * should be used for all of the subordinate state machines (flat or heirarchical)
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
#include "EdgeFollowingSubHSM.h" 
#include "OrientationSubHSM.h"
#include "TravelToBeaconSubHSM.h"
#include "ParallelVatSubHSM.h"
#include "DispenserService.h"
#include "serial.h"
#include "stdio.h"
#include "MotorControl.h"
#include "BumperService.h"
#include "EventCheckers.h"
#include "FindCorrectSideCWSubHSM.h"
#include "DepositBigLoadSubHSM.h"
#include "CheckIfTowerSubHSM.h"
#include "EdgeObstacleSubHSM.h"
#include "FindCorrectSideCCWSubHSM.h"
#include "ReorientateSubHSM.h"
#include "DepositBigLoadCCWSubHSM.h"

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/
//Include any defines you need to do

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/


typedef enum {
    InitState,
    Orientation,
    EdgeFollowing,
    TravelToBeacon,
    ParallelToTower,
    FindCorrectSideCW,
    DepositLoadCW,
    FindCorrectSideCCW,
    DepositLoadCCW,
    Reorientate,
    CheckIfTower,
    EdgeObstacle
} HSMState_t;

static const char *StateNames[] = {
    "InitState",
    "Orientation",
    "EdgeFollowing",
    "TravelToBeacon",
    "ParallelToTower",
    "FindCorrectSideCW",
    "DepositLoadCW",
    "FindCorrectSideCCW",
    "DepositLoadCCW",
    "Reorientate",
    "CheckIfTower",
    "EdgeObstacle"
};

static uint8_t firstTime;
static uint8_t foundCorrectSide;

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES                                                 *
 ******************************************************************************/
/* Prototypes for private functions for this machine. They should be functions
   relevant to the behavior of this state machine
   Example: char RunAway(uint_8 seconds);*/

void InitSubHSM(uint8_t Priority);

/*******************************************************************************
 * PRIVATE MODULE VARIABLES                                                            *
 ******************************************************************************/
/* You will need MyPriority and the state variable; you may need others as well.
 * The type of state variable should match that of enum in header file. */

static HSMState_t CurrentState = InitState; // <- change enum name to match ENUM
static uint8_t MyPriority;
static uint8_t fromUncomfirmedBeacon;

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/

/**
 * @Function InitTemplateHSM(uint8_t Priority)
 * @param Priority - internal variable to track which event queue to use
 * @return TRUE or FALSE
 * @brief This will get called by the framework at the beginning of the code
 *        execution. It will post an ES_INIT event to the appropriate event
 *        queue, which will be handled inside RunTemplateFSM function. Remember
 *        to rename this to something appropriate.
 *        Returns TRUE if successful, FALSE otherwise
 * @author J. Edward Carryer, 2011.10.23 19:25 */
uint8_t InitHSM(uint8_t Priority) {
    MyPriority = Priority;
    // put us into the Initial PseudoState
    CurrentState = InitState;
    //        printf("Init HSM\r\n");
    firstTime = 0;
    //    Beacon_SetMode(ContinuousSampling);
    //    Beacon_StartSampling();
    //    Tank_BeginDetecting();
    // post the initial transition event
    if (ES_PostToService(MyPriority, INIT_EVENT) == TRUE) {
        return TRUE;
    } else {
        return FALSE;
    }
}

/**
 * @Function PostTemplateHSM(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be posted to queue
 * @return TRUE or FALSE
 * @brief This function is a wrapper to the queue posting function, and its name
 *        will be used inside ES_Configure to point to which queue events should
 *        be posted to. Remember to rename to something appropriate.
 *        Returns TRUE if successful, FALSE otherwise
 * @author J. Edward Carryer, 2011.10.23 19:25 */
uint8_t PostHSM(ES_Event ThisEvent) {
    return ES_PostToService(MyPriority, ThisEvent);
}

/**
 * @Function RunTemplateHSM(ES_Event ThisEvent)
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
ES_Event RunHSM(ES_Event ThisEvent) {
    uint8_t makeTransition = FALSE; // use to flag transition
    HSMState_t nextState; // <- change type to correct enum

    ES_Tattle(); // trace call stack

    switch (CurrentState) {
        case InitState: // If current state is initial Pseudo State
            if (ThisEvent.EventType == ES_INIT)// only respond to ES_Init
            {
                InitOrientationSubHSM();
                InitEdgeFollowingSubHSM();
                InitTravelToBeaconSubHSM();
                InitParallelVatSubHSM();
                InitFindCorrectSideCWSubHSM();
                InitFindCorrectSideCCWSubHSM();
                InitDepositBigLoadSubHSM();
                InitDepositBigLoadCCWSubHSM();
                InitReorientateSubHSM();
                InitEdgeObstacleSubHSM();
                nextState = Orientation;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            if (ThisEvent.EventType == FINISHED_MOVING) {
                Motor_SetIndividual(0, 0, 0, 0);
            }
            //            Dispenser_Start();
            break;

        case Orientation: // in the first state, replace this with correct names
            // run sub-state machine for this state
            //NOTE: the SubState Machine runs and responds to events before anything in the this
            //state machine does
            ThisEvent = RunOrientationSubHSM(ThisEvent);
            switch (ThisEvent.EventType) {
                case ES_NO_EVENT:
                    break;
                case DONE:
                    nextState = EdgeFollowing;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;
                default:
                    break;
            }
            break;

        case EdgeFollowing:
            ThisEvent = RunEdgeFollowingSubHSM(ThisEvent);
            switch (ThisEvent.EventType) {
                case ES_NO_EVENT:
                    break;
                case TRAVEL_TO_BEACON:
                    nextState = TravelToBeacon;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;
                case EDGE_OBSTACLE:
                    nextState = EdgeObstacle;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;
                default:
                    break;
            }
            break;
        case TravelToBeacon:
            ThisEvent = RunTravelToBeaconSubHSM(ThisEvent);
            switch (ThisEvent.EventType) {
                case ES_NO_EVENT:
                    break;
                case GO_TO_EDGEFOLLOWING:
                    nextState = EdgeFollowing;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;
                case CHECK_IF_TOWER:
                    nextState = ParallelToTower;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;
                default:
                    break;
            }
            break;

        case ParallelToTower:
            ThisEvent = RunParallelVatSubHSM(ThisEvent);
            switch (ThisEvent.EventType) {
                case ES_NO_EVENT:
                    break;
                case DONE:
                    nextState = FindCorrectSideCW;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;
                default:
                    break;
            }
            break;

        case FindCorrectSideCW:
            if (!getTurningFlag() && ((ThisEvent.EventType == TAPE_ON && ThisEvent.EventParam == FR_INDEX)
                    || (ThisEvent.EventType == BUMPER_PRESSED && ThisEvent.EventParam == FrontBumper))) {
                nextState = FindCorrectSideCCW;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            } else {
                ThisEvent = RunFindCorrectSideCWSubHSM(ThisEvent);
                switch (ThisEvent.EventType) {
                    case ES_NO_EVENT:
                        break;
                    case DONE:
                        printf("got done\r\n");
                        nextState = DepositLoadCW;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                        break;
                    case GIVE_UP:
                        nextState = EdgeFollowing;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                        break;
                    default:
                        break;
                }
            }
            break;

        case DepositLoadCW:
            ThisEvent = RunDepositBigLoadSubHSM(ThisEvent);
            switch (ThisEvent.EventType) {
                case ES_NO_EVENT:
                    break;
                case DONE:
                    nextState = Reorientate;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                default:
                    break;
            }
            break;

        case FindCorrectSideCCW:
            if (ThisEvent.EventType == TAPE_ON && ThisEvent.EventParam == RL_INDEX
                    || (ThisEvent.EventType == BUMPER_PRESSED && ThisEvent.EventParam == RearBumper)) {
                nextState = FindCorrectSideCW;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            } else {
                ThisEvent = RunFindCorrectSideCCWSubHSM(ThisEvent);
                switch (ThisEvent.EventType) {
                    case ES_NO_EVENT:
                        break;
                    case DONE:
                        printf("got done\r\n");
                        nextState = DepositLoadCCW;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                        break;
                    default:
                        break;
                }
            }
            break;

        case DepositLoadCCW:
            ThisEvent = RunDepositBigLoadCCWSubHSM(ThisEvent);
            switch (ThisEvent.EventType) {
                case ES_NO_EVENT:
                    break;
                case DONE:
                    nextState = Reorientate;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                default:
                    break;
            }
            break;

        case Reorientate:
            ThisEvent = RunReorientateSubHSM(ThisEvent);
            switch (ThisEvent.EventType) {
                case ES_NO_EVENT:
                    break;
                case DONE:
                    nextState = EdgeFollowing;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                default:
                    break;
            }
            break;

        case CheckIfTower:
            ThisEvent = RunCheckIfTowerSubHSM(ThisEvent);
            switch (ThisEvent.EventType) {
                case ES_NO_EVENT:
                    break;

                case BEACON_CONFIRMED:
                    nextState = ParallelToTower;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case BEACON_UNCONFIRMED:
                    nextState = Reorientate;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                default:
                    break;
            }
            break;

        case EdgeObstacle:
            if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == OBSTACLE_TIMER) {
                nextState = EdgeFollowing;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            } else {
                ThisEvent = RunEdgeObstacleSubHSM(ThisEvent);
                switch (ThisEvent.EventType) {
                    case ES_NO_EVENT:
                        break;
                    case DONE:
                        nextState = EdgeFollowing;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                        break;
                    case FOUND_SIDE:
                        nextState = DepositLoadCW;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                        break;
                    default:
                        break;
                }
            }
            break;


        default: // all unhandled states fall into here
            break;
    } // end switch on Current State

    if (makeTransition == TRUE) { // making a state transition, send EXIT and ENTRY
        // recursively call the current state with an exit event
        RunHSM(EXIT_EVENT); // <- rename to your own Run function
        CurrentState = nextState;
        RunHSM(ENTRY_EVENT); // <- rename to your own Run function
    }

    ES_Tail(); // trace call stack end
    return ThisEvent;
}


/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/
//void InitSubHSM(uint8_t Priority){
//    
//}