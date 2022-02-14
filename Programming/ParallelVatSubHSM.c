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
#include "BumperService.h"
#include "MotorControl.h"
#include "EventCheckers.h"

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/
typedef enum {
    Start,
    CenterTurn,
    CenterStrafe,
    LeftTurnX,
    RightTurnX,
    StrafeLeft,
    GoBackRight,
    GoBackLeft,
    ForwardY,
    BackwardY,
    LeftTurnY,
    RightTurn2Y
} ParallelVatSubHSMState_t;

static const char *StateNames[] = {
    "Start",
    "CenterTurn",
    "CenterStrafe",
    "LeftTurnX",
    "RightTurnX",
    "StrafeLeft",
    "GoBackRight",
    "GoBackLeft",
    "ForwardY",
    "BackwardY",
    "LeftTurnY",
    "RightTurn2Y"
};

#define DEG_TO_TURN 150
#define CORRECTION_AMOUNT 4


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

static ParallelVatSubHSMState_t CurrentState = Start; // <- change name to match ENUM
static uint8_t MyPriority;


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
uint8_t InitParallelVatSubHSM(void) {
    ES_Event returnEvent;

    CurrentState = Start;
    returnEvent = RunParallelVatSubHSM(INIT_EVENT);
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
ES_Event RunParallelVatSubHSM(ES_Event ThisEvent) {
    uint8_t makeTransition = FALSE; // use to flag transition
    ParallelVatSubHSMState_t nextState; // <- change type to correct enum

    // ES_Tattle(); // trace call stack

    switch (CurrentState) {
        case Start:
            //            printf("Starting SM\r\n");
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    if (Bumper_LastRight() == BUMPER_PRESSED) {
                        nextState = CenterTurn;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                        printf("got center\r\n");
                    } else if (Bumper_LastTowerLeft() == BUMPER_PRESSED && Bumper_LastTowerRight() == BUMPER_PRESSED) {
                        printf("got done\r\n");
                        ThisEvent.EventType = DONE;
                    } else if (Bumper_LastTowerRight() == BUMPER_PRESSED) {
                        nextState = RightTurnX;
                        makeTransition = TRUE;
                        printf("got right\r\n");
                        ThisEvent.EventType = ES_NO_EVENT;
                    } else if (Bumper_LastTowerLeft() == BUMPER_PRESSED) {
                        printf("got left\r\n");
                        nextState = LeftTurnX;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    break;

                case ES_NO_EVENT:
                    break;

                case ES_EXIT:
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;

                default:
                    break;
            }
            break;
        case CenterTurn:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    Motor_SetIndividual(0, 600, 600, 0);
                    Motor_TankLeft();
                    break;
                case BUMPER_PRESSED:
                    if (ThisEvent.EventParam == TowerRightBumper) {
                        nextState = CenterStrafe;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    break;
                case ES_EXIT:
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;
            }
            break;

        case CenterStrafe:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    SetMoveAmount(Move, InchesToTicks(5));
                    Motor_AllDutyCycle(500);
                    Motor_Reverse();
                    break;
                case BUMPER_PRESSED:
                    if (ThisEvent.EventParam == TowerLeftBumper) {
                        nextState = Start;
                        makeTransition = TRUE;
                        ThisEvent.EventType = DONE;
                    }
                    break;
                case FINISHED_MOVING:
                    Motor_SetIndividual(600,600,0,0);
                    Motor_StrafeRight();
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;
                case ES_EXIT:
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;
            }
            break;
        case StrafeLeft:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    SetMoveAmount(Move, InchesToTicks(1));
                    Motor_AllDutyCycle(700);
                    Motor_StrafeLeft();
                    break;

                case ES_NO_EVENT:
                    break;

                case FINISHED_MOVING:
                    nextState = LeftTurnY;
                    makeTransition = TRUE;
                    ThisEvent.EventType = DONE;
                    break;

                default:
                    break;
            }



        case LeftTurnX: // in the first state, replace this with correct names
            //            printf("In LeftTurnX\r\n");
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    printf("got to this state\r\n");
                    SetMoveAmount(DiagTurnL, DegreesToTicks(DEG_TO_TURN));
                    Motor_SetIndividual(0,0,600,600);
                    Motor_StrafeRight();
                    break;
                case ES_NO_EVENT:
                    //                    printf("no\r\n");
                    break;
                case BUMPER_PRESSED:
                    if (ThisEvent.EventParam == TowerRightBumper) {
                        nextState = Start;
                        makeTransition = TRUE;
                        ThisEvent.EventType = DONE;
                    }
                    break;
                case FINISHED_MOVING:
                    nextState = GoBackRight;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case ES_EXIT:
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;

                default:
                    break;
            }
            break;
        case RightTurnX:
            //            printf("In RightTurnX\r\n");
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    SetMoveAmount(DiagTurnR, DegreesToTicks(DEG_TO_TURN));
                    Motor_SetIndividual(600, 600, 0, 0);
                    Motor_StrafeRight();
                    break;
                case ES_NO_EVENT:
                    break;
                case BUMPER_PRESSED:
                    if (ThisEvent.EventParam == TowerLeftBumper) {
                        nextState = Start;
                        makeTransition = TRUE;
                        ThisEvent.EventType = DONE;
                        break;
                    }
                    break;
                case FINISHED_MOVING:
                    nextState = GoBackLeft;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case ES_EXIT:
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;

                default:
                    break;
            }
            break;

        case GoBackRight:
            //            printf("In GoBackRight\r\n");
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    SetMoveAmount(DiagTurnL, DegreesToTicks(DEG_TO_TURN));
                    Motor_SetIndividual(0, 0, 800, 800);
                    Motor_StrafeLeft();
                    break;

                case FINISHED_MOVING:
                    nextState = ForwardY;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case ES_EXIT:
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;

            }
            break;

        case GoBackLeft:
            //            printf("In GobackLeftX\r\n");
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    SetMoveAmount(DiagTurnR, DegreesToTicks(DEG_TO_TURN));
                    Motor_SetIndividual(800, 800, 0, 0);
                    Motor_StrafeLeft();
                    break;

                case FINISHED_MOVING:
                    nextState = BackwardY;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case ES_EXIT:
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;

            }
            break;

        case ForwardY:
            //            printf("In ForwardY\r\n");
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    Motor_AllDutyCycle(800);
                    Motor_Forward();
                    SetMoveAmount(Move, InchesToTicks(CORRECTION_AMOUNT));
                    break;

                case FINISHED_MOVING:
                    nextState = LeftTurnX;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case ES_EXIT:
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;

            }
            break;

        case BackwardY:
            //            printf("In BackwardY\r\n");
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    Motor_AllDutyCycle(800);
                    Motor_Reverse();
                    SetMoveAmount(Move, InchesToTicks(CORRECTION_AMOUNT));
                    break;

                case FINISHED_MOVING:
                    nextState = RightTurnX;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case ES_EXIT:
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;
            }
            break;
        case LeftTurnY:
            //            printf("In LeftTurnY\r\n");
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    SetMoveAmount(Turn, DegreesToTicks(60));
                    Motor_TankLeft();
                    Motor_AllDutyCycle(700);
                    break;

                case FINISHED_TURNING:
                    nextState = RightTurn2Y;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case ES_EXIT:
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;
                case BUMPER_PRESSED:
                    if (ThisEvent.EventParam == TowerRightBumper) {
                        nextState = BackwardY;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    break;
            }
            break;

        case RightTurn2Y:
            //            printf("In RightTurn2Y\r\n");
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    Motor_TankRight();
                    Motor_AllDutyCycle(700);
                    break;

                case ES_EXIT:
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;

                case BUMPER_PRESSED:
                    if (ThisEvent.EventParam == TowerLeftBumper) {
                        nextState = ForwardY;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    break;
            }
            break;
        default: // all unhandled states fall into here
            break;
    } // end switch on Current State

    if (makeTransition == TRUE) { // making a state transition, send EXIT and ENTRY
        // recursively call the current state with an exit event
        RunParallelVatSubHSM(EXIT_EVENT); // <- rename to your own Run function
        CurrentState = nextState;
        RunParallelVatSubHSM(ENTRY_EVENT); // <- rename to your own Run function
    }

    ES_Tail(); // trace call stack end
    return ThisEvent;
}


/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/

