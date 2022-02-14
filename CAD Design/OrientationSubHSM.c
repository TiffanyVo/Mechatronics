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
#include "TapeService.h"
#include "OrientationFSM.h"
#include "ES_Events.h"
#include <BOARD.h>
#include "OrientationSubHSM.h"
#include "MotorControl.h"
#include "EventCheckers.h"
#include "TopLvlHSM.h"

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/
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


#define FORWARD_AMOUNT 5
#define TURN_AMOUNT 45

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES                                                 *
 ******************************************************************************/
/* Prototypes for private functions for this machine. They should be functions
   relevant to the behavior of this state machine */

/*******************************************************************************
 * PRIVATE MODULE VARIABLES                                                            *
 ******************************************************************************/
/* You will need MyPriority and the state variable; you may need others as well.
 * The type of state variable should match that of enum in header file. */

static OrientationFSMState_t CurrentState = CheckForward; // <- change enum name to match ENUM
static uint8_t MyPriority;
static uint8_t frontTapeFlags[2];
static uint8_t BackTapeFlags[2];

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
uint8_t InitOrientationSubHSM(void)
{
    ES_Event returnEvent;

    CurrentState = CheckForward;
    returnEvent = RunOrientationSubHSM(INIT_EVENT);
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
ES_Event RunOrientationSubHSM(ES_Event ThisEvent)
{
    uint8_t makeTransition = FALSE; // use to flag transition
    OrientationFSMState_t nextState; // <- change type to correct enum
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
                    Motor_AllDutyCycle(700); //set duty cycle to 80%
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
                        nextState = CheckForward;
                        makeTransition = TRUE;
                        ThisEvent.EventType = DONE;
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
                    Motor_AllDutyCycle(700); //set duty cycle to 80%
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
                        nextState = CheckForward;
                        makeTransition = TRUE;
                        ThisEvent.EventType = DONE;
                        StopMovement();
                    }
                    break;

                default:
                    break;
            }
            break;
        default: // all unhandled states fall into here
            break;
    } // end switch on Current State

    if (makeTransition == TRUE) { // making a state transition, send EXIT and ENTRY
        // recursively call the current state with an exit event
        RunOrientationSubHSM(EXIT_EVENT); // <- rename to your own Run function
        CurrentState = nextState;
        RunOrientationSubHSM(ENTRY_EVENT); // <- rename to your own Run function
    }

    ES_Tail(); // trace call stack end
    return ThisEvent;
}


/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/

