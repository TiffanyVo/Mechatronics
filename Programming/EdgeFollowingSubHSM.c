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
#include "EdgeFollowingSubHSM.h"
#include "TapeService.h"
#include "OrientationSubHSM.h"
#include "MotorControl.h"
#include "EventCheckers.h"
#include "ES_Timers.h"

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/
typedef enum {
    Init,
    Forward,
    SlowLeft,
    SlowRight,
    WaitForRight,
    Turn180,
    TurnClockwise,
    Adjust,
    Temp,
    ForwardX,
    TurnYAmount,
    Realign,
    TurnUntilBump,
    Finish180,
    WaitForTape
} EdgeFollowingSubHSMState_t;

static const char *StateNames[] = {
    "Init",
    "Forward",
    "SlowLeft",
    "SlowRight",
    "WaitForRight",
    "Turn180",
    "TurnClockwise",
    "Adjust",
    "Temp",
    "ForwardX",
    "TurnYAmount",
    "Realign",
    "TurnUntilBump",
    "Finish180",
    "WaitForTape",
};

#define FORWARD_X_AMOUNT 150
#define TURN_AMOUNT 7
#define CORNER_TIME 5000



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

static EdgeFollowingSubHSMState_t CurrentState = Init; // <- change name to match ENUM
static uint8_t MyPriority;
static uint8_t MyPriority;
static uint8_t clockwiseRight;
static uint8_t clockwiseLeft;
static uint16_t leftDutyCycle, rightDutyCycle;
//static uint8_t right180;
static uint8_t finishedTower;
static uint8_t waitingforTimer;
static uint32_t curCornerTime;
static uint8_t secondCorner;
static uint8_t firstCorner;
static uint8_t CornerReadyFlag;
//static uint8_t onOutsideEdge;
//static uint8_t timeSinceLastBeacon;

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
uint8_t InitEdgeFollowingSubHSM(void) {
    ES_Event returnEvent;

    CurrentState = Init;
    clockwiseRight = 0;
    clockwiseLeft = 0;
    leftDutyCycle = 0;
    rightDutyCycle = 0;
    //    right180 = 0;
    finishedTower = 0;
    waitingforTimer = 0;
    curCornerTime = 0;
    secondCorner = 0;
    firstCorner = 1;
    CornerReadyFlag = 0;
    //    onOutsideEdge = 1;
    //    timeSinceLastBeacon = 0;
    //    returnEvent = RunEdgeFollowingSubHSM(INIT_EVENT); //given code
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
ES_Event RunEdgeFollowingSubHSM(ES_Event ThisEvent) {
    uint8_t makeTransition = FALSE; // use to flag transition
    EdgeFollowingSubHSMState_t nextState; // <- change type to correct enum

    ES_Tattle(); // trace call stack
    //printf("Current State: %d, Current Tape: %d, Current Event: %d \r\n",CurrentState, ThisEvent.EventType, ThisEvent.EventParam );
    switch (CurrentState) {
        case Init:
            //            printf("IN INNIT STATE\r\n");
            CornerReadyFlag = 0;
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    setEdgeFollowing(1);
                    if (CurrentTape(FR_INDEX) == TAPE_OFF && CurrentTape(FL_INDEX) == TAPE_OFF) {
                        nextState = TurnClockwise;
                        //                        printf("started in TurnClockwise\r\n");
                    } else if (CurrentTape(FR_INDEX) == TAPE_OFF) {
                        nextState = SlowLeft;
                        //                        printf("started in SlowLeft\r\n");
                    } else if (CurrentTape(FL_INDEX) == TAPE_OFF) {
                        nextState = SlowRight;
                        //                        printf("started in SlowRight\r\n");
                    } else {
                        nextState = Forward;
                        //                        printf("started in Forward\r\n");
                    }
                    //                    nextState = Forward;
                    ThisEvent.EventType = ES_NO_EVENT;
                    makeTransition = TRUE;
                    break;
                case ES_TIMEOUT:
                    if (ThisEvent.EventParam == EDGE_TIMER) {
                        waitingforTimer = 0;
                    }
                    break;
                default:
                    break;
            }

            break;

        case Forward: // Current state initially set to forward
            //            printf("In forward\r\n");
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    Motor_Forward();
                    Motor_AllDutyCycle(700); //set duty cycle to 80%

                    if (!waitingforTimer) {
                        Beacon_StopSampling();
                        Beacon_SetMode(MovingSample);
                        Beacon_StartSampling();
                    }
                    break;
                case TAPE_OFF:
                    if (ThisEvent.EventParam == FL_INDEX) {
                        nextState = SlowRight;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    if (ThisEvent.EventParam == FR_INDEX) {
                        nextState = SlowLeft;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    break;
                case BEACON_DETECTED:
                    nextState = Init;
                    makeTransition = TRUE;
                    ThisEvent.EventType = TRAVEL_TO_BEACON;
                    //                    timeSinceLastBeacon =  TIMERS_GetTime();
                    break;
                case ES_EXIT:
                    Beacon_StopSampling();
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;
                case BUMPER_PRESSED:
                    if (ThisEvent.EventParam == FrontBumper || ThisEvent.EventParam == TowerLeftBumper) {
                        CurrentState = Forward;
                        //makeTransition = TRUE;
                        ThisEvent.EventType = EDGE_OBSTACLE;
                    }
                    break;

                case ES_TIMEOUT:
                    if (ThisEvent.EventParam == EDGE_TIMER) {
                        waitingforTimer = 0;
                    }
                    break;
                default:
                    break;
            }
            break;

        case SlowRight: //slow right to get back on tape if left tape sensor is off
            //            printf("In SlowRight\r\n");
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    //                    Motor_SlowTurn(700, 400); //slow turn to right
                    Motor_AllDutyCycle(700);
                    Motor_TankRight();
                    break;

                case TAPE_ON:
                    if (ThisEvent.EventParam == FL_INDEX) {
                        nextState = Forward;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    break;
                case TAPE_OFF:
                    if (ThisEvent.EventParam == FR_INDEX) {
                        nextState = WaitForRight;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    break;
                case ES_TIMEOUT:
                    if (ThisEvent.EventParam == EDGE_TIMER) {
                        waitingforTimer = 0;
                    }
                    break;
                case ES_EXIT:
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;
            }
            break;
        case SlowLeft: //slow left to get back on tape if left tape sensor is off
            //            printf("In SlowLeft\r\n");
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    //                    Motor_SlowTurn(400, 700); //slow turn to left
                    Motor_AllDutyCycle(700);
                    Motor_TankLeft();
                    break;

                case TAPE_ON:
                    if (ThisEvent.EventParam == FR_INDEX) {
                        nextState = Forward;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                        break;
                    }
                    break;
                case TAPE_OFF:
                    if (ThisEvent.EventParam == FL_INDEX) {
                        nextState = WaitForRight;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    break;
                case ES_TIMEOUT:
                    if (ThisEvent.EventParam == EDGE_TIMER) {
                        waitingforTimer = 0;
                    }
                    break;
                case ES_EXIT:
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;
            }
            break;

        case WaitForRight:
            //            printf("In WaitForRight\r\n");
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    Motor_AllDutyCycle(500); //set duty cycle to 80%
                    Motor_Forward();

                    if (firstCorner) {
                        curCornerTime = ES_Timer_GetTime();
                        firstCorner = 0;
                    } else {
                        if (CornerReadyFlag && ES_Timer_GetTime() - curCornerTime < CORNER_TIME) {
                            nextState = Forward;
                            makeTransition = TRUE;
                            ThisEvent.EventType = ES_NO_EVENT;
                        }
                        firstCorner = 1;
                        CornerReadyFlag = 1;
                    }
                    //                    if (firstCorner) {
                    //                        firstCorner = 0;
                    //                        //                        curCornerTime = ES_Timer_GetTime();
                    //                        secondCorner = 1;
                    //                    } else if (secondCorner) {
                    //                        curCornerTime = ES_Timer_GetTime();
                    //                    } else if (!secondCorner) {
                    //                        if (ES_Timer_GetTime() - curCornerTime < CORNER_TIME) {
                    //                            nextState = Forward;
                    //                            makeTransition = TRUE;
                    //                            ThisEvent.EventType = ES_NO_EVENT;
                    //                        }
                    //                        secondCorner = 0;
                    //                        firstCorner = 1;
                    //                    }
                    break;
                case TAPE_ON:
                    if (ThisEvent.EventParam == R_INDEX) {
                        nextState = TurnClockwise;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                        break;
                    }
                    if (ThisEvent.EventParam == L_INDEX) {
                        //                        printf("got tape left\r\n");
                        nextState = Turn180;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                        break;
                    }
                    break;
                case ES_TIMEOUT:
                    if (ThisEvent.EventParam == EDGE_TIMER) {
                        waitingforTimer = 0;
                    }
                    break;
                case ES_EXIT:
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;
            }
            break;

        case Turn180: // IF INCONSISTENT, CHECK IF FRONT TAPES CROSS TWICE
            printf("In Turn180\r\n");
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    //                    SetMoveAmount(Turn, DegreesToTicks(180));
                    Motor_AllDutyCycle(600); //set duty cycle to 80%
                    Motor_TankRight();
                    break;

                    //                case FINISHED_TURNING:
                    //                    nextState = Finish180;
                    //                    makeTransition = TRUE;
                    //                    ThisEvent.EventType = ES_NO_EVENT;
                    //                    printf("In Turn180 finished turning\r\n");
                    //                    break;
                case TAPE_ON:
                    if (ThisEvent.EventParam == FL_INDEX) {
                        nextState = Forward;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    break;

                case ES_TIMEOUT:
                    if (ThisEvent.EventParam == EDGE_TIMER) {
                        waitingforTimer = 0;
                    }
                    break;
                case ES_EXIT:
                    printf("in turn180 es exit\r\n");
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;
            }
            break;

            //        case Finish180:
            //            printf("in finish 180\r\n");
            //            switch (ThisEvent.EventParam) {
            //                case ES_ENTRY:
            //                    Motor_AllDutyCycle(900); //set duty cycle to 80%
            //                    Motor_Forward();
            //                    printf("In finish 180- entry\r\n");
            //                    break;
            //
            //                case TAPE_ON:
            //                    if (ThisEvent.EventParam == FR_INDEX) {
            //                        printf("In finish 180- tape event\r\n");
            ////                        nextState = Forward;
            ////                        makeTransition = TRUE;
            //                        ThisEvent.EventType = ES_NO_EVENT;
            //                    }
            //                    break;
            //
            //                case ES_EXIT:
            //                    Motor_AllDutyCycle(0);
            //                    Motor_Reset();
            //                    printf("In finish 180 exit\r\n");
            //                    break;
            //            }
            //
            //            break;

        case TurnClockwise: //corner, move 90 degrees and get back on tape
            //            printf("In TurnClockwise\r\n");
            switch (ThisEvent.EventType) {
                case ES_ENTRY:
                    Motor_AllDutyCycle(500); //set duty cycle to 80%
                    Motor_TankRight(); //only changes direction, not duty cycle speed
                    break;

                case TAPE_ON:
                    if (ThisEvent.EventParam == FL_INDEX) {
                        clockwiseLeft = 1;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    if (ThisEvent.EventParam == FR_INDEX) {
                        clockwiseRight = 1;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    break;

                case ES_EXIT:
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                    break;
                case ES_TIMEOUT:
                    if (ThisEvent.EventParam == EDGE_TIMER) {
                        waitingforTimer = 0;
                    }
                    break;
                default:
                    break;
            }
            if (clockwiseLeft && clockwiseRight) {
                nextState = Forward;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
                clockwiseLeft = 0;
                clockwiseRight = 0;
            }
            break;

        case WaitForTape:
            switch (ThisEvent.EventType) {

                case ES_ENTRY:
                    //                    SetMoveAmount(Move, InchesToTicks(5));
                    Motor_AllDutyCycle(600);
                    Motor_Forward();
                    break;
                case TAPE_ON:
                    if (ThisEvent.EventParam == FR_INDEX || ThisEvent.EventParam == FL_INDEX) {
                        nextState = Forward;
                        makeTransition = TRUE;
                        ThisEvent.EventType = ES_NO_EVENT;
                    }
                    break;

                    //                case FINISHED_MOVING:
                    //                    Motor_AllDutyCycle(0);
                    //                    Motor_Reset();
                    //                    break;

                case ES_EXIT:
                    Motor_AllDutyCycle(0);
                    Motor_Reset();
                default:
                    break;
            }

        default: // all unhandled states fall into here
            break;
    } // end switch on Current State

    if (makeTransition == TRUE) { // making a state transition, send EXIT and ENTRY
        // recursively call the current state with an exit event
        RunEdgeFollowingSubHSM(EXIT_EVENT); // <- rename to your own Run function
        CurrentState = nextState;
        RunEdgeFollowingSubHSM(ENTRY_EVENT); // <- rename to your own Run function
    }

    ES_Tail(); // trace call stack end
    return ThisEvent;
}

/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/

void setEdgeFinishedTower(uint8_t value) {
    finishedTower = value;
}

void setWaitingforTimer(uint8_t value) {
    waitingforTimer = value;
}

uint8_t getWaitingforTimer(void){
    return waitingforTimer;
}