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
#include "FindCorrectSideCWSubHSM.h"
#include "TapeService.h"
#include "OrientationSubHSM.h"
#include "EventCheckers.h"
#include "MotorControl.h"
#include "BumperService.h"

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/
typedef enum {
    CheckFirst,
    MoveAway,
    ForwardX,
    TurnYAmount,
    Realign,
    TurnReverseY,
    ForwardZ,
    Turn120,
    ForwardCenter,
    Parallel,
    StrafeAdjust,
    PushBeforeReturn
} FindCorrectSideCWSubHSMState_t;

static const char *StateNames[] = {
    "CheckFirst",
    "MoveAway",
    "ForwardX",
    "TurnYAmount",
    "Realign",
    "TurnReverseY",
    "ForwardZ",
    "Turn120",
    "ForwardCenter",
    "Parallel",
    "StrafeAdjust",
    "PushBeforeReturn"

};

#define FORWARD_X_AMOUNT 150
#define FORWARD_Z_AMOUNT 800 //750 ticks - 3 inches
#define TURN_120_AMOUNT 200
#define FORWARD_CENTER_AMOUNT 8
#define TURN_AMOUNT 7
#define MOVE_AWAY_AMOUNT 100
#define PARALLEL_AMOUNT 3
#define CORRECT_SIDE_TICK 500
#define PUSH_TIMER_TICK 2000
#define EDGE_TIMER_TICKS 15000
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

static FindCorrectSideCWSubHSMState_t CurrentState; // <- change name to match ENUM
static uint8_t MyPriority;
static uint8_t correctSideFlag;
static uint8_t cornerCount = 0;
static uint8_t TurningFlag = 0;

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
uint8_t InitFindCorrectSideCWSubHSM(void) {
    ES_Event returnEvent;
    correctSideFlag = 0;
    CurrentState = CheckFirst;
    returnEvent = RunFindCorrectSideCWSubHSM(INIT_EVENT);
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
ES_Event RunFindCorrectSideCWSubHSM(ES_Event ThisEvent) {
    uint8_t makeTransition = FALSE; // use to flag transition
    FindCorrectSideCWSubHSMState_t nextState; // <- change type to correct enum

    //    ES_Tattle(); // trace call stack

    switch (CurrentState) {
        case CheckFirst:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    ES_Timer_InitTimer(CORRECT_SIDE_TIMER, CORRECT_SIDE_TICK);
                    //                    Motor_AllDutyCycle(0);
                    //                    Motor_Reset();
                    Tank_BeginDetecting();
                    cornerCount = 0;
                    break;

                case ES_TIMEOUT: // WOULD THIS BE ES_TIMEOUT?
                    if (ThisEvent.EventParam == CORRECT_SIDE_TIMER) {
                        nextState = MoveAway;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                        Tank_StopDetecting();
                    }
                    break;

                case TANK_ON: //DONE, or is this event TANK ON?
                    nextState = CheckFirst;
                    ThisEvent.EventType = DONE;

                    Tank_StopDetecting();
                    break;

                case ES_EXIT:
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    Tank_StopDetecting();
                    break;
            }
            break;

        case MoveAway:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    printf("In moveaway\r\n");
                    SetMoveAmount(Move, MOVE_AWAY_AMOUNT);
                    Motor_AllDutyCycle(700); //set duty cycle to 80%
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
                    Tank_BeginDetecting();
                    if (Bumper_LastTowerLeft() == BUMPER_PRESSED) {
                        nextState = Realign;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    } else {
                        SetMoveAmount(Move, (FORWARD_X_AMOUNT));
                        Motor_AllDutyCycle(500); //set duty cycle to 80%
                        Motor_Forward();
                    }
                    break;
                case FINISHED_MOVING:
                    nextState = TurnYAmount;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;
                case BUMPER_PRESSED:
                    if (ThisEvent.EventParam == TowerLeftBumper) {
                        nextState = Realign;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    break;

                case ES_EXIT:
                    Motor_Reset();
                    Motor_AllDutyCycle(0);
                    Tank_StopDetecting();
                    break;

                default:
                    break;
            }
            break;



        case TurnYAmount:

            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    SetMoveAmount(Turn, DegreesToTicks(TURN_AMOUNT));
                    Motor_AllDutyCycle(500); //set duty cycle to 80%
                    Motor_TankRight();
                    break;

                case ES_NO_EVENT:
                    break;

                case BUMPER_PRESSED:
                    if (ThisEvent.EventParam == TowerLeftBumper) {
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

                default: // all unhandled events pass the event back up to the next level
                    break;
            }
            break;

        case Realign:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    Motor_AllDutyCycle(500); //set duty cycle to 80%
                    Motor_TankLeft();
                    break;

                case BUMPER_RELEASED:
                    if (ThisEvent.EventParam == TowerLeftBumper) {
                        nextState = ForwardX;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                        break;
                    }

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
                    Motor_AllDutyCycle(500); //set duty cycle to 80%
                    Motor_TankLeft();
                    //                    printf("got here\r\n");
                    break;

                case ES_NO_EVENT:
                    break;

                case FINISHED_TURNING:
                    nextState = ForwardZ;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                    //                case BUMPER_PRESSED:
                    //                    if (ThisEvent.EventParam == TowerRightBumper) {
                    //                        nextState = ForwardZ;
                    //                        makeTransition = TRUE;
                    //                        ThisEvent.EventType = ES_NO_EVENT;
                    //                    }
                    //                    break;
                case ES_EXIT:
                    Motor_Reset();
                    Motor_AllDutyCycle(0);
                    break;

                default: // all unhandled events pass the event back up to the next level
                    break;

            }
            break;

        case ForwardZ:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    SetMoveAmount(Move, FORWARD_Z_AMOUNT);
                    Motor_AllDutyCycle(500); //set duty cycle to 80%
                    Motor_Forward();
                    break;

                case ES_NO_EVENT:
                    break;

                case FINISHED_MOVING:
                    nextState = Turn120;
                    makeTransition = TRUE;
                    Motor_Reset();
                    Motor_AllDutyCycle(0);
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case ES_EXIT:
                    Motor_Reset();
                    Motor_AllDutyCycle(0);
                    break;

                default: // all unhandled events pass the event back up to the next level
                    break;

            }
            break;


        case Turn120:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    TurningFlag = 1;
                    cornerCount++;
                    SetMoveAmount(HalfTankL, DegreesToTicks(TURN_120_AMOUNT));
                    //Motor_AllDutyCycle(500); //set duty cycle to 80%
                    Motor_TankRight();
                    Motor_SetIndividual(400, 900, 900, 400);
                    break;

                case ES_NO_EVENT:
                    break;

                case FINISHED_MOVING:
                    nextState = ForwardCenter;
                    makeTransition = TRUE;
                    //                    Motor_Reset();
                    //                    Motor_AllDutyCycle(0);
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case BUMPER_PRESSED:
                    if (ThisEvent.EventParam == TowerLeftBumper) {
                        nextState = ForwardCenter;
                        makeTransition = TRUE;
                        //                        Motor_Reset();
                        //                    Motor_AllDutyCycle(0);
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

        case ForwardCenter:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    SetMoveAmount(Move, InchesToTicks(FORWARD_CENTER_AMOUNT));
                    Motor_AllDutyCycle(500); //set duty cycle to 80%
                    Motor_Forward();
                    break;

                case ES_EXIT:
                    Motor_Reset();
                    Motor_AllDutyCycle(0);
                    break;

                case FINISHED_MOVING:
                    if (cornerCount == 4) {
                        nextState = CheckFirst;
                        makeTransition = TRUE;
                        ThisEvent.EventType = GIVE_UP; //exit back to forward
                        ES_Timer_InitTimer(EDGE_TIMER, EDGE_TIMER_TICKS);

                    } else {
                        nextState = Parallel;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }

                    break;

                default:
                    break;
            }
            break;
        case Parallel:
            //            printf("Tank value: %d\r\n", ThisEvent.EventParam);
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    ES_Timer_InitTimer(PUSH_TIMER, PUSH_TIMER_TICK);
                    Motor_AllDutyCycle(700);
                    if (Bumper_LastTowerRight() == BUMPER_PRESSED) {
                        Motor_SetIndividual(700,700,0,0);
                    }
                    if (Bumper_LastTowerLeft() == BUMPER_PRESSED) {
                        Motor_SetIndividual(0,0,700,700);
                    }
                    if (Bumper_LastTowerLeft() == BUMPER_PRESSED  && Bumper_LastTowerRight() == BUMPER_PRESSED) {
                        Motor_AllDutyCycle(0);
                    }
                    Motor_StrafeRight();
                    //                    SetMoveAmount(Move, InchesToTicks(PARALLEL_AMOUNT));
                    break;

                    //                case FINISHED_MOVING:
                    //                    ES_Timer_InitTimer(CORRECT_SIDE_TIMER, CORRECT_SIDE_TICK);
                    //                    Motor_AllDutyCycle(0);
                    //                    Motor_Reset();
                    //                    Tank_BeginDetecting();
                    //                    break;

                case ES_TIMEOUT: // WOULD THIS BE ES_TIMEOUT?
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

                case TANK_ON: //DONE, or is this event TANK ON?
                    nextState = CheckFirst;
                    makeTransition = TRUE;
                    ThisEvent.EventType = DONE;
                    Tank_StopDetecting();
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
                    TurningFlag = 0;
                    break;
            }
            break;
            //        case Parallel:
            //            //            printf("Tank value: %d\r\n", ThisEvent.EventParam);
            //            switch (ThisEvent.EventType) {
            //                case ES_ENTRY:
            //                    ES_Timer_InitTimer(PUSH_TIMER, PUSH_TIMER_TICK);
            //                    Motor_AllDutyCycle(700);
            //                    Motor_StrafeRight();
            //                    //                    SetMoveAmount(Move, InchesToTicks(PARALLEL_AMOUNT));
            //                    break;
            //
            //                    //                case FINISHED_MOVING:
            //                    //                    ES_Timer_InitTimer(CORRECT_SIDE_TIMER, CORRECT_SIDE_TICK);
            //                    //                    Motor_AllDutyCycle(0);
            //                    //                    Motor_Reset();
            //                    //                    Tank_BeginDetecting();
            //                    //                    break;
            //
            //                case ES_TIMEOUT: // WOULD THIS BE ES_TIMEOUT?
            //                    //                    if (ThisEvent.EventParam == CORRECT_SIDE_TIMER) {
            //                    //                        nextState = MoveAway;
            //                    //                        makeTransition = TRUE;
            //                    //                        ThisEvent.EventType = ES_NO_EVENT;
            //                    //                        Tank_StopDetecting();
            //                    //                    }
            //                    if (ThisEvent.EventParam == PUSH_TIMER) {
            //                        //                        ES_Timer_InitTimer(CORRECT_SIDE_TIMER, CORRECT_SIDE_TICK);
            //                        nextState = StrafeAdjust;
            //                        makeTransition = TRUE;
            //                        ThisEvent.EventType = ES_NO_EVENT;
            //                        //                        Tank_BeginDetecting();
            //                    }
            //                    break;
            //
            //                    //                case TANK_ON: //DONE, or is this event TANK ON?
            //                    //                    nextState = CheckFirst;
            //                    //                    makeTransition = TRUE;
            //                    //                    ThisEvent.EventType = DONE;
            //                    //                    Tank_StopDetecting();
            //                    //                    break;
            //                case BUMPER_PRESSED:
            //                    if (ThisEvent.EventParam == TowerLeftBumper) {
            //                        Motor_SetIndividual(0, 0, 700, 700);
            //                    } else if (ThisEvent.EventParam == TowerRightBumper) {
            //                        Motor_SetIndividual(700, 700, 0, 0);
            //                    }
            //
            //                    if (Bumper_LastTowerLeft() == BUMPER_PRESSED && Bumper_LastTowerRight() == BUMPER_PRESSED) {
            //                        Motor_AllDutyCycle(0);
            //                    }
            //                    break;
            //
            //                case ES_EXIT:
            //                    Motor_AllDutyCycle(0);
            //                    Motor_Reset();
            //                    break;
            //            }
            //            break;
            //
            //        case StrafeAdjust:
            //            switch (ThisEvent.EventType) {
            //                case ES_ENTRY:
            //                    //                    SetMoveAmount(Move, 50);
            //                    ES_Timer_InitTimer(PUSH_TIMER, 300);
            //                    Motor_AllDutyCycle(600);
            //                    Motor_StrafeLeft();
            //                    Tank_BeginDetecting();
            //                    break;
            //
            //                case ES_TIMEOUT:
            //                    if (ThisEvent.EventParam == PUSH_TIMER) {
            //                        Motor_AllDutyCycle(0);
            //                        Motor_Reset();
            //                        ES_Timer_InitTimer(CORRECT_SIDE_TIMER, CORRECT_SIDE_TICK);
            //                        Tank_BeginDetecting();
            //                    }
            //                    if (ThisEvent.EventParam == CORRECT_SIDE_TIMER) {
            //                        //                        nextState = PushBeforeReturn;
            //                        //                        makeTransition = TRUE;
            //                        ThisEvent.EventType = ES_NO_EVENT;
            //                        //                        Tank_StopDetecting();
            //                        Motor_AllDutyCycle(0);
            //                        Motor_Reset();
            //                    }
            //                    break;
            //
            //                case TANK_ON:
            //                    //                    nextState = PushBeforeReturn;
            //                    //                    makeTransition = TRUE;
            //                    ThisEvent.EventType = ES_NO_EVENT;
            //                    //                    correctSideFlag = 1;
            //                    //                    Tank_StopDetecting();
            //                    Motor_AllDutyCycle(0);
            //                    Motor_Reset();
            //                    break;
            //
            //                case ES_EXIT:
            //                    Motor_AllDutyCycle(0);
            //                    Motor_Reset();
            //            }
            //            break;
            //
            //        case PushBeforeReturn:
            //            switch (ThisEvent.EventType) {
            //                case ES_ENTRY:
            //                    ES_Timer_InitTimer(PUSH_TIMER, PUSH_TIMER_TICK);
            //                    Motor_AllDutyCycle(700);
            //                    Motor_StrafeRight();
            //
            //                    break;
            //
            //                case ES_TIMEOUT: // WOULD THIS BE ES_TIMEOUT?
            //                    if (ThisEvent.EventParam == PUSH_TIMER) {
            //                        if (correctSideFlag) {
            //                            nextState = CheckFirst;
            //                            makeTransition = TRUE;
            //                            ThisEvent.EventType = DONE;
            //                            correctSideFlag = 0;
            //                        } else {
            //                            nextState = MoveAway;
            //                            makeTransition = TRUE;
            //                            ThisEvent.EventType = ES_NO_EVENT;
            //                        }
            //                    }
            //                    break;
            //
            //                case BUMPER_PRESSED:
            //                    if (ThisEvent.EventParam == TowerLeftBumper) {
            //                        Motor_SetIndividual(0, 0, 700, 700);
            //                    } else if (ThisEvent.EventParam == TowerRightBumper) {
            //                        Motor_SetIndividual(700, 700, 0, 0);
            //                    }
            //
            //                    if (Bumper_LastTowerLeft() == BUMPER_PRESSED && Bumper_LastTowerRight() == BUMPER_PRESSED) {
            //                        Motor_AllDutyCycle(0);
            //                    }
            //                    break;
            //
            //                case ES_EXIT:
            //                    Motor_AllDutyCycle(0);
            //                    Motor_Reset();
            //                    break;
            //            }
            //            break;

        default: // all unhandled states fall into here
            break;
    } // end switch on Current State

    if (makeTransition == TRUE) { // making a state transition, send EXIT and ENTRY
        // recursively call the current state with an exit event
        RunFindCorrectSideCWSubHSM(EXIT_EVENT); // <- rename to your own Run function
        CurrentState = nextState;
        RunFindCorrectSideCWSubHSM(ENTRY_EVENT); // <- rename to your own Run function
    }

    ES_Tail(); // trace call stack end
    return ThisEvent;
}

uint8_t getTurningFlag(){
    return  TurningFlag;
}
/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/