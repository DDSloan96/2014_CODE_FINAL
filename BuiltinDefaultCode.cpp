/**  2014    2014    2014    2014    2014    2014    2014    2014    2014    2014    2014    2014
 * HHS HHS HHS HHS HHS HHS HHS HHS HHS HHS HHS HHS HHS HHS HHS HHS HHS HHS HHS HHS HHS HHS HHS HHS
 * 2014 Hicksville High School (HHS) Robot Code: Tom Altamura, Daniel Sloan
 * The HHS code is designed to work in the provided  "BuiltinDefaultCode".
 * 
 * The BuiltinDefaultCode extends the IterativeRobot base class.
 * This program prov ides features in the Disabled, Autonomous, and Teleop modes.
 *  
 * Disabled Mode:
 * - Just used to periodically print debug messages.
 * 
 * Autonomous Mode:
 * - xxxx
 * 
 * Teleop Mode:
 * - xxxxx
 *
 * This code assumes the following connections:
 * - Driver Station:
 *   - USB 1 - The "left driver" joystick.  Used as the "left" stick for tank drive
 *   - USB 2 - The "right driver" joystick.  Used as "right" stick for tank drive
 *   - USB 3 - The "operator" joystick.  Used to control kicker and elavating 
 *   - USB 4 - IO Module - with switches and dials 
 *   - USB 5 - Stop Button 
 * 
 * - Robot:
 *   - Digital Sidecar 1:
 *     - PWM 1 - Connected to "left" drive motor - 2 Jaguars w/out "Y" cable
 *     - PWM 2 - Connected to "right" drive motor - 2 Jaguars w/out "Y" cable
 * 
 *   - Digital Sidecar 1:
 *     - Relay 2 - Connected to Spike for the Harvester wheel
 * 
 *   - Digital Sidecar Card 1: (Digital IO 1 - 11, 12-Compressor Limit Sw - 13,14 Encoder Inputs)
 *     - Ch 1 - Autonomous Mode Switch - on Robot
 *     - Ch 2 - Autonomous Mode Switch - on Robot
 *     - Ch 3 - Autonomous Mode Switch - on Robot
 *     - Ch 4 - Autonomous Mode Switch - on Robot
 * 	   - Ch 5 - Ball Capture Limit Switch To Check If Ball Is In The Robot
 *     - Ch 6 - Shooter Out Limit Switch To Check If Ball Is Gone
 *     - Ch 7 - Shooter Reset Limit Switch To Check If Safe To Load
 *     - Ch 8 - Gatherer Limit Switch To See if In Store Or Shoot/Catch position
 * 
 *     - Ch 12 - Compressor Pressure Switch
 *
 *   - Analog Bd 1: (Slot 1 - mounted on top of processor card, w/ jumper for battery voltage readout
 * 
 *   - Solenoid Bd: (Slot 8 on top of 9472 module) tbd; using 12V solenoid via Spikes also
 *     - xxxxx
 * 
 */

#include "WPILib.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "Math.h"
#include "HHS_Includes.h"

/*************************************************************************************************/
//             global variables / macros
/*************************************************************************************************/

//Camera constants used for distance calculation
#define Y_IMAGE_RES 480		//X Image resolution in pixels, should be 120, 240 or 480
#define VIEW_ANGLE 49		//Axis M1013
//#define VIEW_ANGLE 41.7		//Axis 206 camera
//#define VIEW_ANGLE 37.4  //Axis M1011 camera

//Score limits used for target identification
#define RECTANGULARITY_LIMIT 40
#define ASPECT_RATIO_LIMIT 55

//Score limits used for hot target determination
#define TAPE_WIDTH_LIMIT 50
#define VERTICAL_SCORE_LIMIT 50
#define LR_SCORE_LIMIT 50

//Minimum area of particles to be considered
#define AREA_MINIMUM 150

//Maximum number of particles to process
#define MAX_PARTICLES 8

// Variables for autonomous switches on robot - connected to digital inputs 
static int switcha, switchb, switchc, switchd;
static int ltRtPosSw, delaySw, autoTestSw;
static int shooterOutLimitSw, shooterInLimitSw, gathererOpenLimitSw, ballCapturedLimitSw,gatherUpLimitSw=0;
static double autoStartTime, autoCurrentTime, lastPrintTime = 0.0 ;


static int shootingState = 0;
static int gatheringState = 0;
static int catchingState = 0;
static int fixState = 0;

static int AutoMode;

static Threshold threshold(105, 137, 230, 255, 133, 183);
static bool shootNow = false;
static bool fixNow = false;
static bool gatherNow = false;
static bool catchNow = false;
static bool done = true;
static bool abortNow = false;
static bool saveImage = false;
static bool autoShot = false;
static bool inShootingPosition = false, printNow = false;
DigitalInput *m_digIn[(NUM_DIGITAL_INPUTS+1)];
//	DigitalOutput *m_digOut[(NUM_DIGITAL_OUTPUTS+1)];

DriverStationLCD *dsLCD;

class BuiltinDefaultCode : public IterativeRobot
{		
	//Structure to represent the scores for the various tests used for target identification
	struct Scores {
		double rectangularity;
		double aspectRatioVertical;
		double aspectRatioHorizontal;
	};
	
	struct TargetReport {
		int verticalIndex;
		int horizontalIndex;
		bool Hot;
		double totalScore;
		double leftScore;
		double rightScore;
		double tapeWidthScore;
		double verticalScore;
	};
	
	//Drive Base stuff
	RobotDrive *base; 				// robot drive system
	Joystick *m_LeftDriveStick, 	//left drive stick
			 *m_RightDriveStick, 	//right drive stick
			 *m_OpJoystick; 		//operator stick
	
	Talon *m_RightDriveMotor, 		//Right Drivetrain
		  *m_LeftDriveMotor;		//Left Drivetrain
			
	//Harvester Parts
	Relay *m_Harvest;
	
	//Pneumatic parts
	Compressor *m_Compressor;
	static const int NUM_SOLENOIDS=8;
	Solenoid *m_Solenoids[(NUM_SOLENOIDS)+1];//This will allow us to set what we need to set easier with regards to pneumatics
	
	//Vision stuff
	ParticleFilterCriteria2 *criteria;
	Scores *scores;
	TargetReport target;
	int verticalTargets[MAX_PARTICLES];
	int horizontalTargets[MAX_PARTICLES];
	int verticalTargetCount, horizontalTargetCount;
	ColorImage *image;

	DriverStationEnhancedIO *m_ds;

public:
/**
 * Constructor for this "BuiltinDefaultCode" Class.
 * 
 * The constructor creates all of the objects used for the different inputs and outputs of
 * the robot.  Essentially, the constructor defines the input/output mapping for the robot,
 * providing named objects for each of the robot interfaces. 
 */
	BuiltinDefaultCode(void)	{
		printf("BuiltinDefaultCode Constructor Started\n");

		BuiltinDefaultCode::SetPeriod(0.001);  // Need 333 Hz for Talons
		
		//Sets the PWM for the Talons
		m_LeftDriveMotor=new Talon(PWM_LT_MOTOR); //assings Left Motor to LT_MOTOR PWM
		m_RightDriveMotor=new Talon(PWM_RT_MOTOR);//assigns right motor to RT_MOToR PWM
		base = new RobotDrive(m_LeftDriveMotor, m_RightDriveMotor);
		m_ds = &DriverStation::GetInstance()->GetEnhancedIO();
//		base->SetExpiration(0.1);
//		base->SetSafetyEnabled(false);

		//Assigns which USB Each Joystick is
		m_LeftDriveStick = new Joystick(1);  //assigns left drive to USB1
		m_RightDriveStick = new Joystick(2); //assigns right drive to USB2
		m_OpJoystick = new Joystick(3);    //assigns Operator STick to USB3
		

		m_Compressor = new Compressor(CMPRSSR_PRSSR_SW,COMPRESSOR_RELAY); //creates compressor and automatically shuts off when pressure switch activates
		m_Compressor->Start();
		
		//Relay Constructors
		m_Harvest=new Relay(HARVESTER_RELAY);
		
		dsLCD = DriverStationLCD::GetInstance();
		/**** Digital IO Connections - Start ****/

		// TA HHS - Iterate over all the Digital Inputs on the robot, constructing each in turn
		//
		UINT8 digInNum = 1; // start counting digIn at digIn 1
		for (digInNum = 1; digInNum <= NUM_DIGITAL_INPUTS; digInNum++) {
			m_digIn[digInNum] = new DigitalInput(digInNum);
		}

		UINT8 solenoidNum = 1;
		for (solenoidNum = 1; solenoidNum <= NUM_SOLENOIDS; solenoidNum++) {
			m_Solenoids[solenoidNum] = new Solenoid(solenoidNum);
		}
		
		printf("BuiltinDefaultCode Constructor Completed\n");
	}
	
	void RobotInit(void) {
		// Actions which would be performed once (and only once) upon initialization of the
		// robot would be put here.
		printf("Initializing FRC Team 1468 Robot...Running 'RobotInit()'..." );
		printf("Zeroing Shooter Gyro...");

		printf("'RobotInit()' completed.\n");
	}
	
	/********************************** TeleOp Routines *************************************/
	/********************************** TeleOp Routines *************************************/

	void TeleopInit(void) {

//		AutoMode = 0; // reset autonomobus state flag (looked at in near/far kock routines
		base->SetExpiration(1.0);
		base->SetSafetyEnabled(false);
		base->SetLeftRightMotorOutputs(+0.0,+0.0);
				
		// default state - armed to fire
		m_Solenoids[SHOOTER_OUT]->Set(true);
		m_Solenoids[LAUNCHER_OUT]->Set(false);
		m_Solenoids[LAUNCHER_IN]->Set(true);
		m_Solenoids[GATHERER_UP]->Set(true);
		m_Solenoids[GATHERER_OPEN]->Set(false);
		m_Solenoids[GATHERER_CLOSE]->Set(true);
		done = true;		
		shootingState = 0;
				
		printf("TeleopInit() completed.\n");
	}

			
	void TeleopPeriodic(void) {
		/* 
		 * Code placed in here will be called only when a new packet of information
		 * has been received by the Driver Station.  Any code which needs new information
		 * from the DS should go in here
		 */
		
		// increment the number of teleop periodic loops completed
	
		autoCurrentTime = GetClock();
		if ((autoCurrentTime - lastPrintTime) > 0.25)
		{
			 printNow = true;
			 lastPrintTime = autoCurrentTime;
		}
		else 
		{
			 printNow = false;
		}
	
		if (printNow) {
			dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Auto: time = %8.5f", autoCurrentTime);
			dsLCD->UpdateLCD();
			//printf("Auto: time = %8.5f", autoCurrentTime);
		}
		
		HHS_GetDigitalInputs();
		HHS_Drive();
		// in automatic mode
		if ( (m_ds->GetDigital(MANUAL_SWITCH))==0 ){
			if(m_OpJoystick->GetRawButton(OPR_ABORT_BTN)==PRESSED) 
				abortNow = true;
			if (done) {

				if(m_OpJoystick->GetRawButton(OPR_SHOOT_BTN)==PRESSED) {
					shootNow = true;
					done = false;
				}
				if(m_OpJoystick->GetRawButton(OPR_FIXBALL_BTN)==PRESSED){
					fixNow=true;
					done=false;
				}
				else if(m_OpJoystick->GetRawButton(OPR_GATHER_BTN)==PRESSED) {
					gatherNow = true;
					done = false;
				}
				else if(m_OpJoystick->GetRawButton(OPR_CATCH_BTN)==PRESSED) {
					catchNow = true;
					done = false;
				}	
			}else{	
				if (shootNow) 		HHS_AutoShoot();
				else if (gatherNow)	HHS_AutoGather();
				else if (catchNow)	HHS_AutoCatch();
				else if (fixNow) HHS_AutoFix();
			}
		}else{
			HHS_Harvest();
			HHS_Store();
			HHS_Gather();
			if(m_OpJoystick->GetRawButton(OPR_SHOOT_BTN)==PRESSED) {
				m_Solenoids[LAUNCHER_OUT]->Set(true);
				m_Solenoids[LAUNCHER_IN]->Set(false);

			}else{
				m_Solenoids[LAUNCHER_OUT]->Set(false);
				m_Solenoids[LAUNCHER_IN]->Set(true);			
			}
		}	
}			
 // TeleopPeriodic(void)

/********************************** Autonomous Routines *************************************/
/********************************** Autonomous Routines *************************************/
		
	void AutonomousInit(void) {
//		m_autoPeriodicLoops = 0;	
		// Reset the loop counter for autonomous mode
		saveImage=false;
		base->SetSafetyEnabled(false);
//		int autoTimer = 0;
		shootingState = 0;
		
		// default state - armed to fire
		m_Solenoids[SHOOTER_OUT]->Set(true);
		m_Solenoids[LAUNCHER_OUT]->Set(false);
		m_Solenoids[LAUNCHER_IN]->Set(true);
		m_Solenoids[GATHERER_UP]->Set(true);
		m_Solenoids[GATHERER_OPEN]->Set(false);
		m_Solenoids[GATHERER_CLOSE]->Set(true);

		autoStartTime = GetClock();
						
		printf("AutonomousInit() completed.\n");
	}

	void AutonomousPeriodic(void) {
		
		//static int prevAutoMode = 99;

		// Get All Digital Inputs
		HHS_GetDigitalInputs();
		
		if ( (autoTestSw == 1) || true ) 
		{
			HHS_AutonomousPeriodic();
		}
		else { //Test
//			if(prevAutoMode != AutoMode) {
//			printf("Robot DigIn Settings: D=%d, C=%d, B=%d, A=%d AutoMode=%d\n", 
//					switchd, switchc, switchb, switcha, AutoMode);
//			prevAutoMode = AutoMode;
//			}
			switch (AutoMode) {
			case 0:
				HHS_DigInSwitchTest();
				break;
				
			default:
				printf("**** Error Default case - Shouldn't get here!!! Automode = %d\n", AutoMode);
				break;
		} //End Switch;	

		} // End else test
	}

	void HHS_AutonomousPeriodic(void) {

	HHS_GetDigitalInputs();
	
	autoCurrentTime = GetClock();
	if ((autoCurrentTime - lastPrintTime) > 1.0)
	{
		VisionCode();
		 printNow = true;
		 lastPrintTime = autoCurrentTime;
	}
	else 
	{
		 printNow = false;
	}
	if(saveImage==false){
		SaveImage();
		saveImage=true;
	}
	
	if (printNow) {
		dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Auto: time = %8.5f", autoCurrentTime);
		dsLCD->UpdateLCD();
		printf("Shooting Position = %d",inShootingPosition);
		printf("Auto: time = %8.5f Start time=%8.5f /n", autoCurrentTime,autoStartTime);
	}
	
	if ((autoCurrentTime - autoStartTime) < 1.500){
		base->SetLeftRightMotorOutputs(+0.750, +0.750); 
		inShootingPosition = false;
	}else {
		base->SetLeftRightMotorOutputs(+0.0, +0.0); 
		inShootingPosition = true;
	}
	
	// maybe break up the vision code into states to keep processing time down.	
	// also put in timers to see how long it takes
	
	double startVisCode = GetClock();
	double endVisCode = GetClock();
	double timeVisCode = endVisCode - startVisCode;

	if (printNow) {
		printf("Auto: timeVisCode = %8.5f /n", timeVisCode);
	}
	
	shootNow = false;
	
	if(target.Hot){
		if (printNow) {
			dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Hot Target: Proc time = %7.5f", timeVisCode);
			dsLCD->UpdateLCD();
		}
	}
	if 	((target.Hot)&&
		(((ltRtPosSw == LEFT_SIDE) && (target.leftScore > target.rightScore))
		||((ltRtPosSw == RIGHT_SIDE) && (target.rightScore > target.leftScore)))
	)
	
	{
		if (printNow) {
			dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Hot Target Our Side");
			dsLCD->UpdateLCD();
		}
		if (inShootingPosition)
		{
			if (printNow) {
				dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "Engaging Target");
				dsLCD->UpdateLCD();
			}
			shootNow = true;
		}
	}
	if(autoShot==false){	
		HHS_AutoShoot();	//Perform shooting logic
	}	
}	
	
	// Hicksville High School Robot Code
	void HHS_Drive(void){
		static float leftY, rightY;
				
		leftY = -(m_LeftDriveStick->GetY());
		rightY = -(m_RightDriveStick->GetY());

		if (m_LeftDriveStick->GetRawButton(2)) 
			leftY = 0;
		if (m_RightDriveStick->GetRawButton(2)) 
			rightY = 0;
								
		// Drive robot based on joysticks - if both triggers pressed -  direction is reversed 
		// When reversing directions need to switch left and right joystick also!!!
		int reverseDrive = m_LeftDriveStick->GetRawButton(1) && m_RightDriveStick->GetRawButton(1);
		if (reverseDrive == 0)
			base->SetLeftRightMotorOutputs(leftY,rightY);
		else
			base->SetLeftRightMotorOutputs(-rightY,-leftY);
	}
	// Print Drive
	void HHS_PrintDrive(void){
		
		dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "STRING");
		dsLCD->UpdateLCD();
	}
/********************************** Automatic TeleOp Routines *************************************/
/********************************** Automatic TeleOp Routines *************************************/
	
	void HHS_AutoShoot(void){
		static int shootCtr = 0;
		static double startShootTime, currentShootTime ;
		
		shootCtr++;
		
		currentShootTime = GetClock();

		switch (shootingState) {
		case 0:		 	
			if (shootNow)
			{	// loosen up hold of ball before shooting - open catcher / stow and ensure gatherer up
				printf("Shooting stage 1");
				m_Solenoids[GATHERER_UP]->Set(true);			
				// setup timer and goto next state
				startShootTime = GetClock();
				shootingState = 1;
			} 
			break;

		case 1:	// wait to ensure ready to shoot	 
			
			if ( ((currentShootTime - startShootTime) >= 4.0 || (abortNow)||(gatherUpLimitSw==0) )) // is there a feedback to check?
			{	// shoot now by sending shooter pistion out - aarmed and ready!!!
				printf("Shooting stage 2");
				m_Solenoids[GATHERER_OPEN]->Set(true);
				m_Solenoids[GATHERER_CLOSE]->Set(false);
				// setup timer and goto next state
				startShootTime = GetClock();
				shootingState = 2;
			}
			break;

		case 2: // wait until timeout or feedback true
			if ( ((currentShootTime - startShootTime) >= 4.0) || (gathererOpenLimitSw==0) || (abortNow) )
			{	// move both shooter and launcher piston in
				printf("Shooting stage 3");
				printf("shooterOutLimitSw= %d", shooterOutLimitSw);
				printf("abortNow= %d", abortNow);
				
				m_Solenoids[LAUNCHER_OUT]->Set(true);
				m_Solenoids[LAUNCHER_IN]->Set(false);
				// setup timer and goto next state
				startShootTime = GetClock();
				shootingState = 3;
			}
			break;
			
		case 3: // wait until timeout or feedback true
			if ( ((currentShootTime - startShootTime) >= 4.0) || (shooterOutLimitSw == 0) || (abortNow) )
			{	// arm shooter - ready to shoot
				printf("Shooting stage 4");
				m_Solenoids[SHOOTER_OUT]->Set(false);
				m_Solenoids[LAUNCHER_OUT]->Set(false);
				m_Solenoids[LAUNCHER_IN]->Set(true);
				// setup timer and goto next state
				startShootTime = GetClock();
				shootingState = 4;
			}
			break;
			
		case 4:
			if ((shooterInLimitSw==0)|| (abortNow) ){
				m_Solenoids[GATHERER_OPEN]->Set(false);
				m_Solenoids[GATHERER_CLOSE]->Set(true);
				m_Solenoids[SHOOTER_OUT]->Set(true);
				// setup timer and goto next state
				shootingState = 5;
				startShootTime = GetClock();
			}
			break;
			
		case 5: // wait until timeout to be sure
			if ( ((currentShootTime - startShootTime) >= 1.0) || (abortNow) )
			{	// reset state machine - done!
				m_Solenoids[SHOOTER_OUT]->Set(false);
				printf("Shooting stage 5");
				shootingState = 0;
				shootNow = false;
				done = true;
				autoShot=true;
				abortNow = false;
			}
			break;
			
		default:
			printf("**** Error Default case - Shouldn't get here!!! shootingState = %d\n", shootingState);
			break;
		} //End Switch;	
		
			
		
		if ( ((shootCtr % 50) == 0) || (abortNow) ) {
			printf("shootingState %d \n",shootingState);
		}
	}

	
	void HHS_AutoGather(void){
		static int gatherCtr = 0;
		static double startGatherTime, currentGatherTime ;
		
		gatherCtr++;
		
		currentGatherTime = GetClock();

		switch (gatheringState) {
		case 0:		//  	
			if (gatherNow)
			{	// lower gatherer and start motor to bring ball in
				m_Solenoids[GATHERER_CLOSE]->Set(true);
				m_Solenoids[GATHERER_OPEN]->Set(false);
				m_Harvest->Set(Relay::kReverse);
				//lowers whole assembly to bring ball in
				m_Solenoids[GATHERER_UP]->Set(false);

				gatheringState = 1;
			}
			break;

		case 1:		// wait until ball captured 	
			if ( (ballCapturedLimitSw == 0) || (abortNow) )
			{	// when we have the ball - move gatherer up
				m_Solenoids[GATHERER_UP]->Set(true);
				m_Harvest->Set(Relay::kOff);

				startGatherTime = GetClock();
				gatheringState = 2;
			}
			break;

		case 2: // wait until timeout or feedback true
			if (((currentGatherTime - startGatherTime) >= 4.0) || (gatherUpLimitSw == 0) || (abortNow))
			{	// when gatherer up grab onto ball - close catcher and stow 
				m_Harvest->Set(Relay::kOff);

				startGatherTime = GetClock();
				gatheringState = 3;
			}
			break;
			
		case 3: // wait until timeout to be sure
			if (((currentGatherTime - startGatherTime) >= 0.5) || (abortNow))
			{	// reset state machine - done! 
				gatheringState = 0;
				gatherNow = false;
				done = true;
				abortNow = false;
			}
			break;
			
		default:
			printf("**** Error Default case - Shouldn't get here!!! gatheringState = %d\n", gatheringState);
			break;
		} //End Switch;	
		
			
		
		if ((gatherCtr % 50) == 0) {
			printf("gatheringState %d \n",gatheringState);
		}
		
	}

	
	void HHS_AutoCatch(void){
		static int catchCtr = 0;
		static double startCatchTime, currentCatchTime ;
		
		catchCtr++;
		
		currentCatchTime = GetClock();

		switch (catchingState) {
		case 0:		//  	
			if (catchNow)
			{	// ensure gatherer up and start motor and open wide to catch ball 
				m_Solenoids[GATHERER_CLOSE]->Set(false);
				m_Solenoids[GATHERER_OPEN]->Set(true);

				catchingState = 1;
			}
			break;

		case 1:		// wait until ball captured 	
			if ( (ballCapturedLimitSw == 0)&&(gathererOpenLimitSw==0)  || (abortNow))
			{	// when have ball - close catcher and stow 
				m_Solenoids[GATHERER_OPEN]->Set(false);
				m_Solenoids[GATHERER_CLOSE]->Set(true);

				startCatchTime = GetClock();
				catchingState = 2;
			}
			break;

		case 2: // wait until timeout to be sure
			if ( ((currentCatchTime - startCatchTime) >= 0.5)  || (abortNow) )
			{	// reset state machine - done! 
				catchingState = 0;
				catchNow = false;
				done = true;
				abortNow = false;
			}
			break;
			
		default:
			printf("**** Error Default case - Shouldn't get here!!! catchingState = %d\n", catchingState);
			break;
		} //End Switch;	
				
		if ((catchCtr % 50) == 0) {
			printf("catchingState %d \n",catchingState);
		}
	}
	
	void HHS_AutoFix(void){
		static double startFixTime, currentFixTime ;
		
		currentFixTime = GetClock();
		
		switch(fixState){
		case 0:
			if(fixNow){
				m_Solenoids[GATHERER_OPEN]->Set(true);
				m_Solenoids[GATHERER_CLOSE]->Set(false);
				fixState = 1;
			}
			break;
			
		case 1:
			if(gathererOpenLimitSw==0||(abortNow)){
				m_Solenoids[GATHERER_OPEN]->Set(false);
				m_Solenoids[GATHERER_CLOSE]->Set(true);
				fixState = 2;
				startFixTime = GetClock();
			}
			break;
			
		case 2:
			if(currentFixTime-startFixTime>=.5){
				fixState = 0;
				fixNow = false;
				done = true;
				abortNow = false;
			}
			break;
		}
	}
	

/********************************** TeleOp Manual Routines *************************************/
/********************************** TeleOp Manual Routines *************************************/
			

	void HHS_Harvest(void){
		if(m_ds->GetDigital(HARVEST_IN)==0){
			m_Harvest->Set(Relay::kForward);
		}else if(m_ds->GetDigital(HARVEST_OUT)==0){
			m_Harvest->Set(Relay::kReverse);
		}else{
			m_Harvest->Set(Relay::kOff);
		}
	}
	void HHS_Gather(void){
		if(m_ds->GetDigital(GATHER_UP_SWITCH)){
			m_Solenoids[GATHERER_OPEN]->Set(true);
			m_Solenoids[GATHERER_CLOSE]->Set(false);

		}else{
			m_Solenoids[GATHERER_CLOSE]->Set(true);
			m_Solenoids[GATHERER_OPEN]->Set(false);

		}
	}
	
	void HHS_Store(void){
		if(m_ds->GetDigital(STORAGE_UP_SWITCH)){
			printf("gatherer up");
			m_Solenoids[GATHERER_UP]->Set(true);
		}else{
			m_Solenoids[GATHERER_UP]->Set(false);
		}
	}
	
/********************************** End TeleOp Manual Routines *************************************/
/********************************** End TeleOp Manual Routines *************************************/
	
/********************************** Automation Helper Routines *************************************/
/********************************** Automation Helper Routines *************************************/
	
	void HHS_GetDigitalInputs(void){

		shooterOutLimitSw 	= m_digIn[SHTR_OUT_LIMIT_SW]->Get();
		shooterInLimitSw 	= m_digIn[SHTR_IN_LIMIT_SW]->Get();
		gathererOpenLimitSw	= m_digIn[GATHERER_OPEN_LIMIT_SW]->Get();
		ballCapturedLimitSw	= m_digIn[BALL_CAPTURED_LIMIT_SW]->Get();
		gatherUpLimitSw     = m_digIn[GATHERER_UP_LIMIT_SW]->Get();
		// These switches are on the robot and used for autonomous mode only
		switcha	 = m_digIn[AUTO_SW1]->Get(); // 
		switchb	 = m_digIn[AUTO_SW2]->Get(); // 
		switchc	 = m_digIn[AUTO_SW3]->Get(); // 
		switchd	 = m_digIn[AUTO_SW4]->Get(); // 

		ltRtPosSw = switcha;
		delaySw = switchb;
		autoTestSw = switchc;
		
		AutoMode = 8*switchd + 4*switchc + 2*switchb + switcha;
		
		HHS_PrintDsDigInChange();
	}	
	
	void GetPressure(void){
		
	}
	
	void HHS_PrintDsDigInChange() {
		// Check all 8 Driver Station switches 	
		static UINT32 prevDigIn[15], prevDsDigIn[8], currentDsDigIn;
			
		// TA HHS - Iterate over the DS Digital Inputs on the robot, checking each in turn
		int digInNum = 1; // start counting digIn at digIn 1
		for (digInNum = 1; digInNum <= 8; digInNum++) {
			currentDsDigIn = m_ds->GetDigital(digInNum);
			
			if (prevDsDigIn[digInNum] != currentDsDigIn) {
				printf("DsDigIn(%d) = %d was = %d\n", digInNum, currentDsDigIn,
						prevDsDigIn[digInNum]);
				prevDsDigIn[digInNum] = currentDsDigIn;
			}
		}
		
		// TA HHS - Iterate over the Digital Inputs on the robot, checking each in turn
		digInNum = 1; // start counting digIn at digIn 1
		for (digInNum = 1; digInNum <= 10; digInNum++) {
			UINT32 currentDigIn = m_digIn[digInNum]->Get();
			if (prevDigIn[digInNum] != currentDigIn) {
					printf("Robot DigIn Settings: DigIn(%d) = %d was %d\n", 
						digInNum, currentDigIn, prevDigIn[digInNum]);
					prevDigIn[digInNum] = currentDigIn;
			}
		}
	}
	
	void HHS_DigInSwitchTest() {
	// Check all digital inputs switches
	static UINT32 prevDigIn[NUM_DIGITAL_INPUTS+1], currentDigIn;
	base->SetLeftRightMotorOutputs(+0.0, +0.0); 
		
	// TA HHS - Iterate over the Digital Inputs on the robot, checking each in turn
	UINT8 digInNum = 1; // start counting digIn at digIn 1
	for (digInNum = 1; digInNum <= NUM_DIGITAL_INPUTS; digInNum++) {
		currentDigIn = m_digIn[digInNum]->Get();
		if (prevDigIn[digInNum] != currentDigIn) {
			dsLCD->Printf(DriverStationLCD::kUser_Line2, 1,"Robot DigIn Settings: DigIn(%d) = %d was %d\n", 
					digInNum, currentDigIn, prevDigIn[digInNum]);
			dsLCD->UpdateLCD();

			prevDigIn[digInNum] = currentDigIn;
		}
	}
}

/********************************** End Automation Helper Routines *************************************/
/********************************** End Automation Helper Routines *************************************/

	
/********************************** Vision Processing Routines *************************************/
/********************************** Vision Processing Routines *************************************/
		
	
	void VisionCode(void)
	{
		/**
		 * Sample program to use NIVision to find rectangles in the scene that are illuminated
		 * by a LED ring light (similar to the model from FIRSTChoice). The camera sensitivity
		 * is set very low so as to only show light sources and remove any distracting parts
		 * of the image.
		 * 
		 * The CriteriaCollection is the set of criteria that is used to filter the set of
		 * rectangles that are detected. In this example we're looking for rectangles with
		 * a minimum width of 30 pixels and maximum of 400 pixels.
		 * 
		 * The algorithm first does a color threshold operation that only takes objects in the
		 * scene that have a bright green color component. Then a small object filter
		 * removes small particles that might be caused by green reflection scattered from other 
		 * parts of the scene. Finally all particles are scored on rectangularity, and aspect ratio,
		 * to determine if they are a target.
		 *
		 * Look in the VisionImages directory inside the project that is created for the sample
		 * images.
		 */
           /**
             * Do the image capture with the camera and apply the algorithm described above. This
             * sample will either get images from the camera or from an image file stored in the top
             * level directory in the flash memory on the cRIO. The file name in this case is "testImage.jpg"
             */
			//image = new RGBImage("/testImage.jpg");		// get the sample image from the cRIO flash

			
			
			ParticleFilterCriteria2 criteria[] = {
					{IMAQ_MT_AREA, AREA_MINIMUM, 65535, false, false}
			};												
			//Particle filter criteria, used to filter out small particles
			
			AxisCamera &camera = AxisCamera::GetInstance();

			image = camera.GetImage();				//To get the images from the camera comment the line above and uncomment this one
			BinaryImage *thresholdImage = image->ThresholdHSV(threshold);	// get just the green target pixels
			//thresholdImage->Write("/threshold.bmp");
			BinaryImage *filteredImage = thresholdImage->ParticleFilter(criteria, 1);	//Remove small particles
			//filteredImage->Write("Filtered.bmp");

			vector<ParticleAnalysisReport> *reports = filteredImage->GetOrderedParticleAnalysisReports();  //get a particle analysis report for each particle

			verticalTargetCount = horizontalTargetCount = 0;
			//Iterate through each particle, scoring it and determining whether it is a target or not
			if(reports->size() > 0)
			{
				scores = new Scores[reports->size()];
				for (unsigned int i = 0; i < MAX_PARTICLES && i < reports->size(); i++) {
					ParticleAnalysisReport *report = &(reports->at(i));
					
					//Score each particle on rectangularity and aspect ratio
					scores[i].rectangularity = scoreRectangularity(report);
					scores[i].aspectRatioVertical = scoreAspectRatio(filteredImage, report, true);
					scores[i].aspectRatioHorizontal = scoreAspectRatio(filteredImage, report, false);			
					
					//Check if the particle is a horizontal target, if not, check if it's a vertical target
					if(scoreCompare(scores[i], false))
					{
						printf("particle: %d  is a Horizontal Target centerX: %d  centerY: %d \n", i, report->center_mass_x, report->center_mass_y);
						horizontalTargets[horizontalTargetCount++] = i; //Add particle to target array and increment count
					} else if (scoreCompare(scores[i], true)) {
						printf("particle: %d  is a Vertical Target centerX: %d  centerY: %d \n", i, report->center_mass_x, report->center_mass_y);
						verticalTargets[verticalTargetCount++] = i;  //Add particle to target array and increment count
					} else {
						printf("particle: %d  is not a Target centerX: %d  centerY: %d \n", i, report->center_mass_x, report->center_mass_y);
					}
					printf("Scores rect: %f  ARvert: %f \n", scores[i].rectangularity, scores[i].aspectRatioVertical);
					printf("ARhoriz: %f  \n", scores[i].aspectRatioHorizontal);	
				}

				//Zero out scores and set verticalIndex to first target in case there are no horizontal targets
				target.totalScore = target.leftScore = target.rightScore = target.tapeWidthScore = target.verticalScore = 0;
				target.verticalIndex = verticalTargets[0];
				for (int i = 0; i < verticalTargetCount; i++)
				{
					ParticleAnalysisReport *verticalReport = &(reports->at(verticalTargets[i]));
					for (int j = 0; j < horizontalTargetCount; j++)
					{
						ParticleAnalysisReport *horizontalReport = &(reports->at(horizontalTargets[j]));
						double horizWidth, horizHeight, vertWidth, leftScore, rightScore, tapeWidthScore, verticalScore, total;
	
						//Measure equivalent rectangle sides for use in score calculation
						imaqMeasureParticle(filteredImage->GetImaqImage(), horizontalReport->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE, &horizWidth);
						imaqMeasureParticle(filteredImage->GetImaqImage(), verticalReport->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &vertWidth);
						imaqMeasureParticle(filteredImage->GetImaqImage(), horizontalReport->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &horizHeight);
						
						//Determine if the horizontal target is in the expected location to the left of the vertical target
						leftScore = ratioToScore(1.2*(verticalReport->boundingRect.left - horizontalReport->center_mass_x)/horizWidth);
						//Determine if the horizontal target is in the expected location to the right of the  vertical target
						rightScore = ratioToScore(1.2*(horizontalReport->center_mass_x - verticalReport->boundingRect.left - verticalReport->boundingRect.width)/horizWidth);
						//Determine if the width of the tape on the two targets appears to be the same
						tapeWidthScore = ratioToScore(vertWidth/horizHeight);
						//Determine if the vertical location of the horizontal target appears to be correct
						verticalScore = ratioToScore(1-(verticalReport->boundingRect.top - horizontalReport->center_mass_y)/(4*horizHeight));
						total = leftScore > rightScore ? leftScore:rightScore;
						total += tapeWidthScore + verticalScore;
						
						//If the target is the best detected so far store the information about it
						if(total > target.totalScore)
						{
							target.horizontalIndex = horizontalTargets[j];
							target.verticalIndex = verticalTargets[i];
							target.totalScore = total;
							target.leftScore = leftScore;
							target.rightScore = rightScore;
							target.tapeWidthScore = tapeWidthScore;
							target.verticalScore = verticalScore;
						}
					}
					//Determine if the best target is a Hot target
					target.Hot = hotOrNot(target);
				}
				
				if(verticalTargetCount > 0)
				{
					//Information about the target is contained in the "target" structure
					//To get measurement information such as sizes or locations use the
					//horizontal or vertical index to get the particle report as shown below
					ParticleAnalysisReport *distanceReport = &(reports->at(target.verticalIndex));
					double distance = computeDistance(filteredImage, distanceReport);
					if(target.Hot)
					{
						printf("Hot target located \n");
						printf("Distance: %f \n", distance);
					} else {
						printf("No hot target present \n");
						printf("Distance: %f \n", distance);
					}
				}
			}

			// be sure to delete images after using them
			delete filteredImage;
			delete thresholdImage;
			delete image;
			
			//delete allocated reports and Scores objects also
			delete scores;
			delete reports;
		
	}
	
	/**
	 * Computes the estimated distance to a target using the height of the particle in the image. For more information and graphics
	 * showing the math behind this approach see the Vision Processing section of the ScreenStepsLive documentation.
	 * 
	 * @param image The image to use for measuring the particle estimated rectangle
	 * @param report The Particle Analysis Report for the particle
	 * @return The estimated distance to the target in feet.
	 */
	double computeDistance (BinaryImage *image, ParticleAnalysisReport *report) {
		double rectLong, height;
		int targetHeight;
		
		imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE, &rectLong);
		//using the smaller of the estimated rectangle long side and the bounding rectangle height results in better performance
		//on skewed rectangles
		height = min(report->boundingRect.height, rectLong);
		targetHeight = 32;
		
		return Y_IMAGE_RES * targetHeight / (height * 12 * 2 * tan(VIEW_ANGLE*PI/(180*2)));
	}
	
	/**
	 * Computes a score (0-100) comparing the aspect ratio to the ideal aspect ratio for the target. This method uses
	 * the equivalent rectangle sides to determine aspect ratio as it performs better as the target gets skewed by moving
	 * to the left or right. The equivalent rectangle is the rectangle with sides x and y where particle area= x*y
	 * and particle perimeter= 2x+2y
	 * 
	 * @param image The image containing the particle to score, needed to perform additional measurements
	 * @param report The Particle Analysis Report for the particle, used for the width, height, and particle number
	 * @param outer	Indicates whether the particle aspect ratio should be compared to the ratio for the inner target or the outer
	 * @return The aspect ratio score (0-100)
	 */
	double scoreAspectRatio(BinaryImage *image, ParticleAnalysisReport *report, bool vertical){
		double rectLong, rectShort, idealAspectRatio, aspectRatio;
		idealAspectRatio = vertical ? (4.0/32) : (23.5/4);	//Vertical reflector 4" wide x 32" tall, horizontal 23.5" wide x 4" tall
		
		imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE, &rectLong);
		imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &rectShort);
		
		//Divide width by height to measure aspect ratio
		if(report->boundingRect.width > report->boundingRect.height){
			//particle is wider than it is tall, divide long by short
			aspectRatio = ratioToScore(((rectLong/rectShort)/idealAspectRatio));
		} else {
			//particle is taller than it is wide, divide short by long
			aspectRatio = ratioToScore(((rectShort/rectLong)/idealAspectRatio));
		}
		return aspectRatio;		//force to be in range 0-100
	}
	
	/**
	 * Compares scores to defined limits and returns true if the particle appears to be a target
	 * 
	 * @param scores The structure containing the scores to compare
	 * @param vertical True if the particle should be treated as a vertical target, false to treat it as a horizontal target
	 * 
	 * @return True if the particle meets all limits, false otherwise
	 */
	bool scoreCompare(Scores scores, bool vertical){
		bool isTarget = true;

		isTarget &= scores.rectangularity > RECTANGULARITY_LIMIT;
		if(vertical){
			isTarget &= scores.aspectRatioVertical > ASPECT_RATIO_LIMIT;
		} else {
			isTarget &= scores.aspectRatioHorizontal > ASPECT_RATIO_LIMIT;
		}

		return isTarget;
	}
	
	/**
	 * Computes a score (0-100) estimating how rectangular the particle is by comparing the area of the particle
	 * to the area of the bounding box surrounding it. A perfect rectangle would cover the entire bounding box.
	 * 
	 * @param report The Particle Analysis Report for the particle to score
	 * @return The rectangularity score (0-100)
	 */
	double scoreRectangularity(ParticleAnalysisReport *report){
		if(report->boundingRect.width*report->boundingRect.height !=0){
			return 100*report->particleArea/(report->boundingRect.width*report->boundingRect.height);
		} else {
			return 0;
		}	
	}	
	
	/**
	 * Converts a ratio with ideal value of 1 to a score. The resulting function is piecewise
	 * linear going from (0,0) to (1,100) to (2,0) and is 0 for all inputs outside the range 0-2
	 */
	double ratioToScore(double ratio){
		return (max(0, min(100*(1-fabs(1-ratio)), 100)));
	}
	
	/**
	 * Takes in a report on a target and compares the scores to the defined score limits to evaluate
	 * if the target is a hot target or not.
	 * 
	 * Returns True if the target is hot. False if it is not.
	 */
	bool hotOrNot(TargetReport target){
		bool isHot = true;
		
		isHot &= target.tapeWidthScore >= TAPE_WIDTH_LIMIT;
		isHot &= target.verticalScore >= VERTICAL_SCORE_LIMIT;
		isHot &= (target.leftScore > LR_SCORE_LIMIT) | (target.rightScore > LR_SCORE_LIMIT);
		
		return isHot;
	}
	
/********************************** End Vision Processing Routines *************************************/
/********************************** End Vision Processing Routines *************************************/

	
/********************************** Disabled Routines *************************************/
/********************************** Disabled Routines *************************************/
	
	void DisabledInit(void) {
//		m_disabledPeriodicLoops = 0;
		//pulled from HHS_Diababled init
		printf("\n\n");
		printf("Hicksville High School \n");
		printf("FIRST Robotics Team 1468\n");


		printf("DisabledInit() completed.\n");
		// Move the cursor down a few, since we'll move it back up in periodic.
		printf("\x1b[2B");

		dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Time: %4.1f", GetClock());
		dsLCD->UpdateLCD();

	}
	
	void DisabledPeriodic(void)  {
		static const INT32 startSec = (INT32)GetClock();// Reset the loop counter for disabled mode
		static INT32 printSec = (INT32)GetClock() + 1;
		// while disabled, printout the duration of current disabled mode in seconds
		if (GetClock() > printSec) {
			// Move the cursor back to the previous line and clear it.
			if(((printSec-startSec)%5==0))
				printf("Disabled seconds: %d\r", printSec - startSec);			
			printSec++;
		}
	}	
	void SaveImage(void){
		
		AxisCamera &camera=AxisCamera::GetInstance();
		image = camera.GetImage();		//To get the images from the camera comment the line above and uncomment this one
		image->Write("/Original.bmp");
		BinaryImage *thresholdImage = image->ThresholdHSV(threshold);	// get just the green target pixels
		thresholdImage->Write("/threshold.bmp");
		//BinaryImage *filteredImage = thresholdImage->ParticleFilter(criteria, 1);	//Remove small particles
		//filteredImage->Write("Filtered.bmp");
	}
};  // end of Class

START_ROBOT_CLASS(BuiltinDefaultCode);

