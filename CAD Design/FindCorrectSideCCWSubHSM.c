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
#include "FindCorrectSideCCWSubHSM.h"
#include "TapeService.h"
#include "OrientationSubHSM.h"
#include "EventCheckers.h"
#include "MotorControl.h"
#include "BumperService.h"

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/
typedef enum {
    FixReverse,
    PushTower,
    MoveAway,
    ForwardX,
    TurnYAmount,
    Realign,
    TurnReverseY,
    ForwardZ,
    Turn120,
    ForwardCenter,
    Parallel
} FindCorrectSideCCWSubHSMState_t;

static const char *StateNames[] = {
    "FixReverse",
    "PushTower",
    "MoveAway",
    "ForwardX",
    "TurnYAmount",
    "Realign",
    "TurnReverseY",
    "ForwardZ",
    "Turn120",
    "ForwardCenter",
    "Parallel"
};

#define FORWARD_X_AMOUNT 150
#define FORWARD_Z_AMOUNT 3
#define TURN_120_AMOUNT 200
#define TURN_AMOUNT 7
#define MOVE_AWAY_AMOUNT 100
#define PARALLEL_AMOUNT 3
#define CORRECT_SIDE_TICK 500
#define FORWARD_CENTER_AMOUNT 9                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
#define PUSH_TIMER_TICK 2000

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

static FindCorrectSideCCWSubHSMState_t CurrentState;
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
uint8_t InitFindCorrectSideCCWSubHSM(void) {
    ES_Event returnEvent;

    CurrentState = FixReverse;
    returnEvent = RunFindCorrectSideCCWSubHSM(INIT_EVENT);
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
ES_Event RunFindCorrectSideCCWSubHSM(ES_Event ThisEvent) {
    uint8_t makeTransition = FALSE; // use to flag transition
    FindCorrectSideCCWSubHSMState_t nextState; // <- change type to correct enum

    //ES_Tattle(); // trace call stack

    switch (CurrentState) {
        case FixReverse:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    SetMoveAmount(Move, InchesToTicks(3));
                    Motor_AllDutyCycle(600); //set duty cycle to 80%
                    Motor_Reverse();
                    break;

                case FINISHED_MOVING:
                    nextState = PushTower;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case ES_EXIT:
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;
            }

            break;
        case PushTower:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    ES_Timer_InitTimer(PUSH_TIMER, PUSH_TIMER_TICK);
                    Motor_AllDutyCycle(700);
                    Motor_StrafeRight();
                    if (Bumper_LastTowerRight() == BUMPER_PRESSED) {
                        Motor_SetIndividual(700, 700, 0, 0);
                    }
                    if (Bumper_LastTowerLeft() == BUMPER_PRESSED) {
                        Motor_SetIndividual(0, 0, 700, 700);
                    }
                    if (Bumper_LastTowerRight() == BUMPER_PRESSED && Bumper_LastTowerLeft() == BUMPER_PRESSED) {
                        Motor_SetIndividual(0, 0, 0, 0);
                    }
                    break;

                case ES_TIMEOUT:
                    if (ThisEvent.EventParam == CORRECT_SIDE_TIMER) {
                        nextState = MoveAway;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                        Tank_StopDetecting();
                    }
                    if (ThisEvent.EventParam == PUSH_TIMER) {
                        ES_Timer_InitTimer(CORRECT_SIDE_TIMER, CORRECT_SIDE_TICK);
                        Motor_AllDutyCycle(0);
                        Motor_Reset();
                        Tank_BeginDetecting();
                    }
                    break;

                case BUMPER_PRESSED:
                    if (ThisEvent.EventParam == TowerLeftBumper) {
                        Motor_SetIndividual(0, 0, 700, 700);
                    } else if (ThisEvent.EventParam == TowerRightBumper) {
                        Motor_SetIndividual(700, 700, 0, 0);
                    }

                    if (Bumper_LastTowerLeft() == BUMPER_PRESSED && Bumper_LastTowerRight() == BUMPER_PRESSED) {
                        Motor_AllDutyCycle(0);
                    }
                    break;
                case ES_EXIT:
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;
            }
            break;

        case MoveAway:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    SetMoveAmount(Move, MOVE_AWAY_AMOUNT);
                    Motor_AllDutyCycle(700);
                    Motor_StrafeLeft();
                    break;

                case FINISHED_MOVING:
                    nextState = ForwardX;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;
                case ES_EXIT:
                    Motor_Reset();
                    Motor_AllDutyCycle(0);
                    break;
                default:
                    break;
            }
            break;

        case ForwardX:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    if (Bumper_LastTowerRight() == BUMPER_PRESSED) {
                        nextState = Realign;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    } else {
                        SetMoveAmount(Move, (FORWARD_X_AMOUNT));
                        Motor_AllDutyCycle(500);
                        Motor_Reverse();
                    }
                    break;
                case FINISHED_MOVING:
                    nextState = TurnYAmount;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;
                case BUMPER_PRESSED:
                    if (ThisEvent.EventParam == TowerRightBumper) {
                        nextState = Realign;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    break;
                case ES_EXIT:
                    Motor_Reset();
                    Motor_AllDutyCycle(0);
                    break;

                default: // all unhandled events pass the event back up to the next level
                    break;
            }
            break;

        case TurnYAmount:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    SetMoveAmount(Turn, DegreesToTicks(TURN_AMOUNT));
                    Motor_AllDutyCycle(500);
                    Motor_TankLeft();
                    break;

                case ES_NO_EVENT:
                    break;

                case BUMPER_PRESSED:
                    if (ThisEvent.EventParam == TowerRightBumper) {
                        nextState = Realign;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                        StopMovement();
                    }
                    break;

                case FINISHED_TURNING:
                    nextState = TurnReverseY;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case ES_EXIT:
                    Motor_Reset();
                    Motor_AllDutyCycle(0);
                    break;

                default:
                    break;
            }
            break;

        case Realign:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    Motor_AllDutyCycle(500);
                    Motor_TankRight();
                    break;

                case BUMPER_RELEASED:
                    if (ThisEvent.EventParam == TowerRightBumper) {
                        nextState = ForwardX;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    break;

                case ES_EXIT:
                    Motor_Reset();
                    Motor_AllDutyCycle(0);
                    break;
            }
            break;

        case TurnReverseY:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    SetMoveAmount(Turn, DegreesToTicks(TURN_AMOUNT));
                    Motor_AllDutyCycle(500);
                    Motor_TankRight();
                    break;

                case ES_NO_EVENT:
                    break;

                case FINISHED_TURNING:
                    nextState = ForwardZ;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case ES_EXIT:
                    Motor_Reset();
                    Motor_AllDutyCycle(0);
                    break;

                default:
                    break;
            }
            break;

        case ForwardZ:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    SetMoveAmount(Move, InchesToTicks(FORWARD_Z_AMOUNT));
                    Motor_AllDutyCycle(500);
                    Motor_Reverse();
                    break;

                case ES_NO_EVENT:
                    break;

                case FINISHED_MOVING:
                    Motor_Reset();
                    Motor_AllDutyCycle(0);
                    nextState = Turn120;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case ES_EXIT:
                    Motor_Reset();
                    Motor_AllDutyCycle(0);
                    break;

                default:
                    break;
            }
            break;

        case Turn120:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    SetMoveAmount(HalfTankR, DegreesToTicks(TURN_120_AMOUNT));
                    Motor_TankLeft();
                    Motor_SetIndividual(400, 900, 900, 400);
                    break;
                case ES_NO_EVENT:
                    break;
                case FINISHED_MOVING:
                    nextState = ForwardCenter;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;
                case BUMPER_PRESSED:
                    if (ThisEvent.EventParam == TowerRightBumper) {
                        nextState = ForwardCenter;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    break;
                case ES_EXIT:
                    Motor_Reset();
                    Motor_AllDutyCycle(0);
                    break;
                default:
                    break;
            }
            break;
        case ForwardCenter:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    SetMoveAmount(Move, InchesToTicks(FORWARD_CENTER_AMOUNT));
                    Motor_AllDutyCycle(500);
                    Motor_Reverse();
                    break;

                case ES_EXIT:
                    Motor_Reset();
                    Motor_AllDutyCycle(0);
                    break;
                case FINISHED_MOVING:
                    nextState = Parallel;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;
                default:
                    break;
            }
            break;

        case Parallel:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    ES_Timer_InitTimer(PUSH_TIMER, PUSH_TIMER_TICK);
                    Motor_AllDutyCycle(700);
                    Motor_StrafeRight();
                    if (Bumper_LastTowerRight() == BUMPER_PRESSED) {
                        Motor_SetIndividual(700, 700, 0, 0);
                    }
                    if (Bumper_LastTowerLeft() == BUMPER_PRESSED) {
                        Motor_SetIndividual(0, 0, 700, 700);
                    }
                    if (Bumper_LastTowerRight() == BUMPER_PRESSED && Bumper_LastTowerLeft() == BUMPER_PRESSED) {
                        Motor_SetIndividual(0, 0, 0, 0);
                    }

                    //                SetMoveAmount(Move, InchesToTicks(PARALLEL_AMOUNT));
                    break;
                    //            case FINISHED_MOVING:
                    //                ES_Timer_InitTimer(CORRECT_SIDE_TIMER, CORRECT_SIDE_TICK);
                    //                Motor_AllDutyCycle(0);
                    //                Motor_Reset();
                    //                Tank_BeginDetecting();
                    //                break;
                case ES_TIMEOUT:
                    if (ThisEvent.EventParam == CORRECT_SIDE_TIMER) {
                        nextState = MoveAway;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                        Tank_StopDetecting();
                    }
                    if (ThisEvent.EventParam == PUSH_TIMER) {
                        ES_Timer_InitTimer(CORRECT_SIDE_TIMER, CORRECT_SIDE_TICK);
                        Motor_AllDutyCycle(0);
                        Motor_Reset();
                        Tank_BeginDetecting();
                    }
                    break;

                case BUMPER_PRESSED:
                    if (ThisEvent.EventParam == TowerLeftBumper) {
                        Motor_SetIndividual(0, 0, 700, 700);
                    } else if (ThisEvent.EventParam == TowerRightBumper) {
                        Motor_SetIndividual(700, 700, 0, 0);
                    }

                    if (Bumper_LastTowerLeft() == BUMPER_PRESSED && Bumper_LastTowerRight() == BUMPER_PRESSED) {
                        Motor_AllDutyCycle(0);
                    }
                    break;
                case TANK_ON:
                    nextState = FixReverse;
                    makeTransition = TRUE;
                    ThisEvent.EventType = DONE;
                    Tank_StopDetecting();
                    break;
                case ES_EXIT:
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;
            }
            break;

        default: // all unhandled states fall into here
            break;
    } // end switch on Current State

    if (makeTransition == TRUE) { // making a state transition, send EXIT and ENTRY
        // recursively call the current state with an exit event
        RunFindCorrectSideCCWSubHSM(EXIT_EVENT); // <- rename to your own Run function
        CurrentState = nextState;
        RunFindCorrectSideCCWSubHSM(ENTRY_EVENT); // <- rename to your own Run function
    }

    ES_Tail(); // trace call stack end
    return ThisEvent;
}


/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/

