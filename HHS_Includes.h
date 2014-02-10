/*******************************************************************************
* FILE NAME: HHS_include.hpp
*
* DESCRIPTION: 
*  This is the Hicksville HS include file which corresponds to HHS_code.c
*  It contains some aliases and function prototypes used in those files.
*
* USAGE:
*  If you add your own routines to HHS_code.c, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*
* By: Tom Altamura, xxx, yyy
*******************************************************************************/
#ifndef _HHS_INCLUDE_H
#define _HHS_INCLUDE_H

//Version Number
#define VER_NUM ="2014.01"

/*******************************************************************************
                            MACRO DECLARATIONS
*******************************************************************************/
/* Add your macros (aliases and constants) here.                              */
/* Do not edit the ones in ifi_aliases.h                                      */
/* Macros are substituted in at compile time and make your code more readable */
/* as well as making it easy to change a constant value in one place, rather  */
/* than at every place it is used in your code.                               */
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/


//Below is where we will define which switch controls what function
//add more functions as needed, we will add I/O numbers at a later time

//**********************************************//
//         Operator Controls                    //
//**********************************************//

// Automatic Control vs individual button manual override control 2 POS SW
#define GATHER_UP_SWITCH    1
#define MANUAL_SWITCH 	    3
#define CATCH_SWITCH        4
#define SHOOTER_LOAD_SWITCH 5
#define STORAGE_UP_SWITCH   7
//switch to turn on harvester
#define HARVEST_IN 		    11
#define HARVEST_OUT 	    13
//switch to catch

// Gatherer Motor - 3 pos switch

// Gatherer Up/Down pneumatic switch

//Driver station switches
#define PRESSED 1 //This is the value we will use when a button is pressed or a switch is on(Binary)

#define OPR_SHOOT_BTN 	1
#define OPR_CATCH_BTN 	2
#define OPR_GATHER_BTN 	3
#define OPR_FIXBALL_BTN 4
#define OPR_ABORT_BTN 	7

//**********************************************//
//         Motor and other PWM Controls         //
//**********************************************//

//THis refers to which PWM the talons will be connected to(3 way split)
//These are on Digital IO Board
#define PWM_LT_MOTOR 	1
#define PWM_RT_MOTOR 	2

//**********************************************//
//			Digital IO Positions				//
//**********************************************//

// Digital IO Switches On Robot - Note that there are 14 digital inputs available
// #12 is for the Compressor Limit Switch, and 13,14 are for the Encoder

// This is the number of Dig IOs we have direct control over (not Compressor Limit Sw or Encoder(s)
#define NUM_DIGITAL_INPUTS  10

//Have a 4 Switch Box for Automomous Routines
#define AUTO_SW1					1				//
#define AUTO_SW2					2				//
#define AUTO_SW3					3				//
#define AUTO_SW4					4				//

//Limit Switches
#define BALL_CAPTURED_LIMIT_SW		5
#define SHTR_OUT_LIMIT_SW			6				//
#define SHTR_IN_LIMIT_SW			7				//
#define GATHERER_OPEN_LIMIT_SW		8				//
#define CATCHER_LIMIT_SW 			9
#define GATHERER_UP_LIMIT_SW		10
#define CMPRSSR_PRSSR_SW			11				//
//
#define LEFT_SIDE					0
#define RIGHT_SIDE					1

//**********************************************//
//         Automation Routines                  //
//**********************************************//

//This will allow us to select which Autonomous Routine we want to do
//#define AUTO_SELECT 1

//Limit switches will be placed below
//#define PRESSURE_SWITCH 6
//#define
//Any other sensor data will be defined below at a later date

//**********************************************//
//         Relays on Digital IO                 //
//**********************************************//
#define HARVESTER_RELAY 	2
#define COMPRESSOR_RELAY 	8

//**********************************************//
//    Solenoids for Pneumatics on NI 9472 Card  //
//**********************************************//
						
#define LAUNCHER_OUT 	1  //Launcher fired
#define LAUNCHER_IN     2  //launcher is in down position
#define CATCHER_OPEN    3  //Opens Catcher Assembly
#define CATCHER_CLOSED  4  //Closes Catcher Assembly
#define GATHERER_OPEN	5  //brings harvester assembly up
#define GATHERER_CLOSE	6  //brings harvester assembly down
#define SHOOTER_OUT		7  //loads shooter
#define GATHERER_UP     8  //raises whole assembly

//**********************************************//
//                 Constants                    //
//**********************************************//
#define PI 3.1415926535897932384626433832795
//#define WHEEL_DIA 4.0 //wheel diameter in inches
//#define WHEEL_CIRCUMFERENCE 12.566370614359172953850573533118 //This is assuming the 4 inch wheel doesnt change
#define COUNTS_PER_INCH			  38
#define COUNTS_PER_FEET			  (12*COUNTS_PER_INCH)
//Define any other constants we need below

//**********************************************//
//                 Functions                    //
//**********************************************//


void HHS_Drive(void);
void HHS_Harvest(void);
void HHS_Store(void);
void HHS_Gather(void);
void HHS_AutoFix(void);
void HHS_AutoShoot(void);
void HHS_AutoGather(void);
void HHS_AutoCatch(void);
void HHS_AutonomousPeriodic(void);
void HHS_PrintDrive(void);
void HHS_GetDigitalInputs(void);
void GetPressure(void);

//Print Routines
void HHS_PrintDsDigInChange(void);
void HHS_DigInSwitchTest(void);

#endif