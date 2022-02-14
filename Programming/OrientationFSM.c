/*
 * File: TemplateFSM.c
 * Author: J. Edward Carryer
 * Modified: Gabriel H Elkaim
 *
 * Template file to set up a Flat State Machine to work with the Events and Services
 * Frameword (ES_Framework) on the Uno32 for the CMPE-118/L class. Note that this file
 * will need to be modified to fit your exact needs, and most of the names will have
 * to be changed to match your code.
 *
 * This is provided as an example and a good place to start.
 *
 *Generally you will just be modifying the statenames and the run function
 *However make sure you do a find and replace to convert every instance of
 *  "Template" to your current state machine's name
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
#include "TapeService.h"
#include "OrientationFSM.h"
#include <BOARD.h>
//Uncomment these for the Roaches
//#include "roach.h"
//#include "RoachFrameworkEvents.h"
#include <stdio.h>
#include "MotorControl.h"
#include "EventCheckers.h"
#include "TopLvlHSM.h"


/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/


/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES                                                 *
 ******************************************************************************/
/* Prototypes for private functions for this machine. They should be functions
   relevant to the behavior of this state machine.*/


/*******************************************************************************
 * PRIVATE MODULE VARIABLES                                                            *
 ******************************************************************************/

/* You will need MyPriority and the state variable; you may need others as well.
 * The type of state variable should match that of enum in header file. */

typedef enum {
    CheckForward,
    ReverseX,
    SmallTurn,
    TankTurnRight,
    StrafeLeft,
    Done

} OrientationFSMState_t;

static const char *StateNames[] = {
    "CheckForward",
    "ReverseX",
    "SmallTurn",
    "TankTurnRight",
    "StrafeLeft",
    "Done"
};


static OrientationFSMState_t CurrentState = CheckForward; // <- change enum name to match ENUM
static uint8_t MyPriority;
static uint8_t frontTapeFlags[2];
static uint8_t BackTapeFlags[2];

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/

/**
 * @Function InitTemplateFSM(uint8_t Priority)
 * @param Priority - internal variable to track which event queue to use
 * @return TRUE or FALSE
 * @brief This will get called by the framework at the beginning of the code
 *        execution. It will post an ES_INIT event to the appropriate event
 *        queue, which will be handled inside RunTemplateFSM function. Remember
 *        to rename this to something appropriate.
 *        Returns TRUE if successful, FALSE otherwise
 * @author J. Edward Carryer, 2011.10.23 19:25 */
uint8_t InitOrientationFSM(uint8_t Priority) {
    MyPriority = Priority;
    // put us into the Initial PseudoState
    CurrentState = CheckForward;

    // post the initial transition event
    if (ES_PostToService(MyPriority, INIT_EVENT) == TRUE) {
        return TRUE;
    } else {
        return FALSE;
    }
}

/**
 * @Function PostTemplateFSM(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be posted to queue
 * @return TRUE or FALSE
 * @brief This function is a wrapper to the queue posting function, and its name
 *        will be used inside ES_Configure to point to which queue events should
 *        be posted to. Remember to rename to something appropriate.
 *        Returns TRUE if successful, FALSE otherwise
 * @author J. Edward Carryer, 2011.10.23 19:25 */
uint8_t PostOrientationFSM(ES_Event ThisEvent) {
    return ES_PostToService(MyPriority, ThisEvent);
}

/**
 * @Function RunTemplateFSM(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be responded.
 * @return Event - return event (type and param), in general should be ES_NO_EVENT
 * @brief This function is where you implement the whole of the flat state machine,
 *        as this is called any time a new event is passed to the event queue. This
 *        function will be called recursively to implement the correct order for a
 *        state transition to be: exit current state -> enter next state using the
 *        ES_EXIT and ES_ENTRY events.
 * @note Remember to rename to something appropriate.
 *       Returns ES_NO_EVENT if the event have been "consumed."
 * @author J. Edward Carryer, 2011.10.23 19:25 */

#define FORWARD_AMOUNT 10
#define TURN_AMOUNT 45

ES_Event RunOrientationFSM(ES_Event ThisEvent) {
    uint8_t makeTransition = FALSE; // use to flag transition
    OrientationFSMState_t nextState; // <- need to change enum type here
    ES_Event thisEvent;
    ES_EventTyp_t curEvent;
    //ES_Tattle(); // trace call stack

    switch (CurrentState) {
        case CheckForward: // If current state is initial Psedudo State

            switch (ThisEvent.EventType) {
                case(ES_ENTRY):
                    setEdgeFollowing(0);
                    frontTapeFlags[0] = 0;
                    frontTapeFlags[1] = 0;
                    SetMoveAmount(Move, InchesToTicks(FORWARD_AMOUNT));
                    Motor_AllDutyCycle(700); //set duty cycle to 80%
                    Motor_Forward();
                    break;

                case(ES_EXIT):
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;

                case(TAPE_ON):
                    if (ThisEvent.EventParam == FR_INDEX) {
                        frontTapeFlags[1] = 1;
                    }
                    if (ThisEvent.EventParam == FL_INDEX) {
                        frontTapeFlags[0] = 1;
                    }
                    if (frontTapeFlags[0] && frontTapeFlags[1]) {
                        nextState = TankTurnRight;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                        StopMovement();
                    }
                    break;

                case(FINISHED_MOVING):
                    nextState = ReverseX;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                default:
                    break;
            }
            break;

        case ReverseX:
            switch (ThisEvent.EventType) {
                case(ES_ENTRY):
                    SetMoveAmount(Move, InchesToTicks(FORWARD_AMOUNT));
                    Motor_AllDutyCycle(700); //set duty cycle to 80%
                    Motor_Reverse();
                    break;

                case(ES_EXIT):
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;

                case(FINISHED_MOVING):
                    nextState = SmallTurn;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                default:
                    break;
            }
            break;

        case SmallTurn:
            switch (ThisEvent.EventType) {
                case(ES_ENTRY):
                    SetMoveAmount(Turn, DegreesToTicks(TURN_AMOUNT));
                    Motor_AllDutyCycle(700); //set duty cycle to 80%
                    Motor_TankRight();
                    break;

                case(ES_EXIT):
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;

                case(FINISHED_TURNING):
                    nextState = CheckForward;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                default:
                    break;
            }
            break;

        case TankTurnRight:
            switch (ThisEvent.EventType) {
                case(ES_ENTRY):
                    Motor_AllDutyCycle(500); //set duty cycle to 80%
                    Motor_TankRight();
                    break;

                case(ES_EXIT):
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;

                case(TAPE_OFF):
                    if (ThisEvent.EventParam == FR_INDEX) {
                        nextState = StrafeLeft;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                        StopMovement();
                    }
                    break;
                    
                case(TAPE_ON):
                    if (ThisEvent.EventParam == RR_INDEX) {
                        nextState = Done;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                        StopMovement();
                    }
                    break;

                default:
                    break;
            }
            break;

        case StrafeLeft:
            switch (ThisEvent.EventType) {
                case(ES_ENTRY):
                    Motor_AllDutyCycle(500); //set duty cycle to 80%
                    Motor_StrafeLeft();
                    break;

                case(ES_EXIT):
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;

                case(TAPE_ON):
                    if (ThisEvent.EventParam == FR_INDEX) {
                        nextState = TankTurnRight;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                        StopMovement();
                    }
                    if (ThisEvent.EventParam == RR_INDEX) {
                        nextState = Done;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                        StopMovement();
                    }
                    //                    if (ThisEvent.EventParam == R_INDEX) {
                    //                        nextState = Done;
                    //                        makeTransition = TRUE;
                    //                        ThisEvent.EventType = ES_NO_EVENT;
                    //                        StopMovement();
                    //                    }
                    break;

                default:
                    break;
            }
            break;

        case Done:
            curEvent = DONE;
            thisEvent.EventType = curEvent;
            makeTransition = TRUE;
            break;
        default: // all unhandled states fall into here
            break;
    } // end switch on Current State
    if (makeTransition == TRUE) { // making a state transition, send EXIT and ENTRY
        // recursively call the current state with an exit event
        RunOrientationFSM(EXIT_EVENT);
        CurrentState = nextState;
        RunOrientationFSM(ENTRY_EVENT);
    }
    ES_Tail(); // trace call stack end
    return ThisEvent;
}


/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/
