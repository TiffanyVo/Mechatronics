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
#include "DepositBigLoadCCWSubHSM.h"
#include "TapeService.h"
#include "OrientationSubHSM.h"
#include "MotorControl.h"
#include "EventCheckers.h"
#include "BumperService.h"
#include "DispenserService.h"
#include "EdgeFollowingSubHSM.h"

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/
typedef enum {
    ForwardX,
    TurnYAmount,
    Realign,
    TurnReverseY,
    ForwardAdjust,
    PushTower,
    Deposit,
    Reverse,
    Shimmy
} DepositBigLoadCCWSubHSMState_t;

static const char *StateNames[] = {
    "ForwardX",
    "TurnYAmount",
    "Realign",
    "TurnReverseY",
    "ForwardAdjust",
    "PushTower",
    "Deposit",
    "Reverse",
    "Shimmy"
};

#define REVERSE_AMOUNT 1200
#define FORWARD_ADJUST_AMOUNT 175
#define PUSH_AMOUNT 250
#define FORWARD_X_AMOUNT 75
#define TURN_AMOUNT 5
#define EDGE_TIMER_TICKS 15000
#define SHIMMY_TIMER_TICKS 100
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

static DepositBigLoadCCWSubHSMState_t CurrentState = Reverse; // <- change name to match ENUM
static uint8_t MyPriority;

static uint8_t shimmyCounter;

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
uint8_t InitDepositBigLoadCCWSubHSM(void) {
    ES_Event returnEvent;
    shimmyCounter = 0;
    CurrentState = Reverse;
    returnEvent = RunDepositBigLoadCCWSubHSM(INIT_EVENT);
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
ES_Event RunDepositBigLoadCCWSubHSM(ES_Event ThisEvent) {
    uint8_t makeTransition = FALSE;
    DepositBigLoadCCWSubHSMState_t nextState;

    //ES_Tattle(); // trace call stack

    switch (CurrentState) {
        case Reverse:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    SetMoveAmount(Move, REVERSE_AMOUNT);
                    Motor_AllDutyCycle(500); //set duty cycle to 80%
                    Motor_Reverse();
                    break;

                case FINISHED_MOVING:
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    nextState = ForwardX;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case ES_EXIT:
                    Motor_Reset();
                    Motor_AllDutyCycle(0);
                    break;
            }
            break;

        case ForwardX:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
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
                    break;

                case TOWER_ON_TAPE:
                    //                    printf("On tape\r\n");
                    nextState = ForwardAdjust;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    //                    Motor_Reset();
                    //                    Motor_AllDutyCycle(0);
                    //                    printf("Tower Left: %d\r\n", ThisEvent.EventParam);
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

                case TOWER_ON_TAPE:
                    //                    printf("On tape\r\n");
                    nextState = ForwardAdjust;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    //                    Motor_Reset();
                    //                    Motor_AllDutyCycle(0);
                    //                    printf("Tower Left: %d\r\n", ThisEvent.EventParam);
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

                case TOWER_ON_TAPE:
                    //                    printf("On tape\r\n");
                    nextState = ForwardAdjust;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    //                    Motor_Reset();
                    //                    Motor_AllDutyCycle(0);
                    //                    printf("Tower Left: %d\r\n", ThisEvent.EventParam);
                    break;
            }
            break;
        case TurnReverseY:

            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    SetMoveAmount(Turn, DegreesToTicks(TURN_AMOUNT));
                    Motor_AllDutyCycle(500); //set duty cycle to 80%
                    Motor_TankLeft();
                    break;

                case ES_NO_EVENT:
                    break;

                case FINISHED_TURNING:
                    nextState = ForwardX;
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

        case ForwardAdjust:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    SetMoveAmount(Move, FORWARD_ADJUST_AMOUNT);
                    Motor_AllDutyCycle(400); //set duty cycle to 80%
                    Motor_Forward();
                    break;

                case FINISHED_MOVING:
                    //                    Motor_AllDutyCycle(0);
                    //                    Motor_Reset();
                    nextState = PushTower;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;
                    //                case TOWER_OFF_TAPE:
                    //                    Motor_Reset();
                    //                    Motor_AllDutyCycle(0);
                    //                    nextState = PushTower;
                    //                    makeTransition = TRUE;
                    //                    ThisEvent.EventType = ES_NO_EVENT;
                    //                    break;

                case ES_EXIT:
                    Motor_Reset();
                    Motor_AllDutyCycle(0);
                    break;
            }
            break;

        case PushTower:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    //                    SetMoveAmount(Move, PUSH_AMOUNT);
                    Motor_AllDutyCycle(500); //set duty cycle to 80%
                    Motor_StrafeRight();
                    ES_Timer_InitTimer(CORRECT_SIDE_TIMER, 2000);
                    break;

                    //                case FINISHED_MOVING:
                    //                    Motor_AllDutyCycle(0);
                    //                    Motor_Reset();
                    //                    nextState = Deposit;
                    //                    makeTransition = TRUE;
                    //                    ThisEvent.EventType = ES_NO_EVENT;
                    //                    break;

                case ES_EXIT:
                    Motor_Reset();
                    Motor_AllDutyCycle(0);
                    break;

                case ES_TIMEOUT:
                    if (ThisEvent.EventParam == CORRECT_SIDE_TIMER) {
                        Motor_AllDutyCycle(0);
                        Motor_Reset();
                        nextState = Deposit;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
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

            }
            break;

        case Deposit:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    Dispenser_Start();
                    break;

                case DISPENING_FINISHED:
                    nextState = Shimmy;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case ES_EXIT:
                    Motor_Reset();
                    Motor_AllDutyCycle(0);
                    ES_Timer_InitTimer(EDGE_TIMER, EDGE_TIMER_TICKS);
                    setWaitingforTimer(1);
                    setEdgeFinishedTower(1);

                    break;
            }
            break;

        case Shimmy:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    shimmyCounter = 0;
                    ES_Timer_InitTimer(SHIMMY_TIMER, SHIMMY_TIMER_TICKS);
                    Motor_Reverse();
                    Motor_AllDutyCycle(700);
                    break;

                case ES_TIMEOUT:
                    if (shimmyCounter > 4) {
                        nextState = Reverse;
                        makeTransition = TRUE;
                        ThisEvent.EventType = DONE;
                    } else if (shimmyCounter % 2) {
                        ES_Timer_InitTimer(SHIMMY_TIMER, SHIMMY_TIMER_TICKS);
                        Motor_Reverse();
                        Motor_AllDutyCycle(700);
                        shimmyCounter++;
                    } else {
                        ES_Timer_InitTimer(SHIMMY_TIMER, SHIMMY_TIMER_TICKS);
                        Motor_Forward();
                        Motor_AllDutyCycle(700);
                        shimmyCounter++;
                    }
                    break;

                case ES_EXIT:
                    Motor_Reset();
                    Motor_AllDutyCycle(0);
                    break;
            }
            break;
    } // end switch on Current State

    if (makeTransition == TRUE) { // making a state transition, send EXIT and ENTRY
        // recursively call the current state with an exit event
        RunDepositBigLoadCCWSubHSM(EXIT_EVENT); // <- rename to your own Run function
        CurrentState = nextState;
        RunDepositBigLoadCCWSubHSM(ENTRY_EVENT); // <- rename to your own Run function
    }

    ES_Tail(); // trace call stack end
    return ThisEvent;
}


/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/
