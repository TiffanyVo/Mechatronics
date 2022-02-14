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
#include "EdgeObstacleSubHSM.h"
#include "MotorControl.h"
#include "EventCheckers.h"
#include "TapeService.h"
#include "EdgeFollowingSubHSM.h"

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/
typedef enum {
    InitPSubState,
    Temp,
    Closer,
    ReverseX,
    Realign,
    TurnUntilBump,
    ReverseAdjust,
    TurnRight,
    StrafeLeft
} EdgeObstacleSubHSMState_t;

static const char *StateNames[] = {
    "InitPSubState",
    "Temp",
    "Closer",
    "ReverseX",
    "Realign",
    "TurnUntilBump",
    "ReverseAdjust",
    "TurnRight",
    "StrafeLeft"
};

#define TURN_AMOUNT 7
#define REVERSE_X_AMOUNT 150
#define OBSTACLE_TIMER_TICKS 30000
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

static EdgeObstacleSubHSMState_t CurrentState = InitPSubState; // <- change name to match ENUM
static uint8_t MyPriority;
static uint8_t detectedTank;

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
uint8_t InitEdgeObstacleSubHSM(void) {
    ES_Event returnEvent;

    CurrentState = Temp;
    detectedTank = 0;
    returnEvent = RunEdgeObstacleSubHSM(INIT_EVENT);
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
ES_Event RunEdgeObstacleSubHSM(ES_Event ThisEvent) {
    uint8_t makeTransition = FALSE; // use to flag transition
    EdgeObstacleSubHSMState_t nextState; // <- change type to correct enum

    // ES_Tattle(); // trace call stack

    switch (CurrentState) {
        case InitPSubState: // If current state is initial Psedudo State
            if (ThisEvent.EventType == ES_INIT)// only respond to ES_Init
            {
                // this is where you would put any actions associated with the
                // transition from the initial pseudo-state into the actual
                // initial state

                nextState = Temp;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
            }
            break;


        case Temp:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    printf("got in to obstacle\r\n");
                    ES_Timer_InitTimer(OBSTACLE_TIMER, OBSTACLE_TIMER_TICKS);
                    Beacon_StopSampling();

                    setEdgeFollowing(0);
                    SetMoveAmount(Move, InchesToTicks(2));
                    Motor_AllDutyCycle(500); //set duty cycle to 80%
                    Motor_Reverse();
                    break;

                case FINISHED_MOVING:
                    SetMoveAmount(Turn, DegreesToTicks(80));
                    Motor_AllDutyCycle(500); //set duty cycle to 80%
                    Motor_TankLeft();
                    break;
                case FINISHED_TURNING:
                    nextState = Closer;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case ES_EXIT:
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;
                    
                case ES_TIMEOUT:
                    if (ThisEvent.EventParam == EDGE_TIMER) {
                        setWaitingforTimer(0);
                    }
                    break;
                default:
                    break;
            }
            break;


        case Closer:
            printf("in closer\r\n");
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    SetMoveAmount(DiagTurnR, DegreesToTicks(90));
                    Motor_SetIndividual(500, 500, 700, 700);
                    Motor_StrafeRight();
                    break;

                case FINISHED_MOVING:
                    nextState = TurnRight;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                    break;

                case BUMPER_PRESSED:
                    if ((ThisEvent.EventParam == TowerRightBumper)) {
                        nextState = Realign;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    break;

                case ES_EXIT:
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;

                case ES_TIMEOUT:
                    if (ThisEvent.EventParam == EDGE_TIMER) {
                        setWaitingforTimer(0);
                    }
                    break;

                default:
                    break;
            }
            break;

        case ReverseX:
            printf("In ForwardX\r\n");
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    SetMoveAmount(Move, (REVERSE_X_AMOUNT));
                    Motor_AllDutyCycle(500); //set duty cycle to 80%
                    Motor_Reverse();
                    Tank_BeginDetecting();
                    break;

                case FINISHED_MOVING:
                    nextState = TurnUntilBump;
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
                    Tank_StopDetecting();
                    detectedTank = 0;
                    break;

                case TAPE_ON:
                    if (ThisEvent.EventParam == RR_INDEX) {
                        nextState = ReverseAdjust;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    break;

                case ES_TIMEOUT:
                    if (ThisEvent.EventParam == EDGE_TIMER) {
                        setWaitingforTimer(0);
                    }
                    break;

                case TOWER_ON_TAPE:
                    if (detectedTank == 1) {
                        if (!getWaitingforTimer()) {
                            CurrentState = Temp;
                            //makeTransition = TRUE;
                            ThisEvent.EventType = FOUND_SIDE;
                        }
                    }
                    break;

                case TANK_ON:
                    detectedTank = 1;
                    if (CurrentTowerTape() == TOWER_ON_TAPE) {
                        CurrentState = Temp;
                        //makeTransition = TRUE;
                        ThisEvent.EventType = FOUND_SIDE;
                    }
                    break;

                default:
                    break;
            }
            break;

        case Realign:
            printf("In Realign\r\n");
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    Motor_AllDutyCycle(500); //set duty cycle to 80%
                    Motor_TankRight();
                    break;

                case BUMPER_RELEASED:
                    if (ThisEvent.EventParam == TowerRightBumper) {
                        nextState = ReverseX;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                        break;
                    }

                case ES_EXIT:
                    Motor_Reset();
                    Motor_AllDutyCycle(0);
                    break;

                case TAPE_ON:
                    if (ThisEvent.EventParam == RR_INDEX) {
                        nextState = ReverseAdjust;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    break;

                case ES_TIMEOUT:
                    if (ThisEvent.EventParam == EDGE_TIMER) {
                        setWaitingforTimer(0);
                    }
                    break;
            }
            break;

        case TurnUntilBump:
            printf("In TurnUntilBump: Event: %d\r\n", ThisEvent.EventType);
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    Motor_TankLeft();
                    Motor_SetIndividual(400, 900, 900, 400);
                    break;

                case ES_NO_EVENT:
                    break;

                case BUMPER_PRESSED:
                    if (ThisEvent.EventParam == TowerRightBumper) {
                        nextState = Realign;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    break;

                case TAPE_ON:
                    if (ThisEvent.EventParam == RR_INDEX) {
                        nextState = ReverseAdjust;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    break;

                case ES_EXIT:
                    Motor_Reset();
                    Motor_AllDutyCycle(0);
                    break;

                case ES_TIMEOUT:
                    if (ThisEvent.EventParam == EDGE_TIMER) {
                        setWaitingforTimer(0);
                    }
                    break;

                default: // all unhandled events pass the event back up to the next level
                    break;

            }
            break;

        case TurnRight:
            printf("in turn180\r\n");
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    SetMoveAmount(Turn, DegreesToTicks(360));
                    Motor_AllDutyCycle(500); //set duty cycle to 80%
                    Motor_TankLeft();
                    break;

                case FINISHED_TURNING:
                    CurrentState = Temp;
                    //makeTransition = TRUE;
                    ThisEvent.EventType = DONE;
                    break;

                case TAPE_ON:
                    if (ThisEvent.EventParam == FR_INDEX) {
                        CurrentState = Temp;
                        //makeTransition = TRUE;
                        ThisEvent.EventType = DONE;
                    }
                    break;

                case ES_EXIT:
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;

                case ES_TIMEOUT:
                    if (ThisEvent.EventParam == EDGE_TIMER) {
                        setWaitingforTimer(0);
                    }
                    break;
                default:
                    break;
            }
            break;

        case ReverseAdjust:
            printf("reverse adjust\r\n");
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    Motor_AllDutyCycle(500); //set duty cycle to 80%
                    Motor_Reverse();
                    break;

                case ES_EXIT:
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;

                case ES_TIMEOUT:
                    if (ThisEvent.EventParam == EDGE_TIMER) {
                        setWaitingforTimer(0);
                    }
                    break;

                case TAPE_ON:
                    if (ThisEvent.EventParam == L_INDEX || ThisEvent.EventParam == R_INDEX) {
                        nextState = StrafeLeft;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    break;

                default:
                    break;
            }
            break;

        case StrafeLeft:
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    SetMoveAmount(Move, InchesToTicks(3));
                    Motor_AllDutyCycle(500);
                    Motor_StrafeLeft();
                    break;

                case TAPE_ON:
                    if (ThisEvent.EventParam == L_INDEX) {
                        nextState = TurnRight;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    break;
                case FINISHED_MOVING:
                    nextState = TurnRight;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
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
        RunEdgeObstacleSubHSM(EXIT_EVENT); // <- rename to your own Run function
        CurrentState = nextState;
        RunEdgeObstacleSubHSM(ENTRY_EVENT); // <- rename to your own Run function
    }

    ES_Tail(); // trace call stack end
    return ThisEvent;
}


/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/

