

/*     VERSION HISTORY
*           First number in the version number should only increase if the overall structure of
*                    the code changes. The code should remain functionally the same. If the
*                   first number changes, there should be no other changes to the code and the
*                    version should be a single number (ie. V2  or V15       not V2.1)
*
*           Second number in the version number should only change if a feature is added to the
*                    code. Removal of a feature (so long as its not the last feature added)
*                    should be treated like a change to the overall structure.
*
*           Third number should change only for bug fixes and minor "cosmetic" changes. Use of
*                    this number should be avoided by only commiting robot code that works
*                    propperly.
*
*          !!!!!!!!!!UPDATE VERSION HISTORY BEFORE COMMIT!!!!!!!!!!
*            !!!!!!!!!!UPDATE VERSION HISTORY BEFORE COMMIT!!!!!!!!!!
*        !!!!!!!!!!UPDATE VERSION HISTORY BEFORE COMMIT!!!!!!!!!!
*
*        Version  |  Developer   |   Comments About Changes
*        _________|______________|___________________________________________________________________________________________________________
*         V1      |  RAT         |   Created the framework for the robot code 
*         V1.1    |  RAT         |   Added some code to accomodate a single action pnumatics directly controled by either bumper
*         V1.2    |  RAT         |   Added code to swap between high and low sensitivity and added code to use for timer functions
*         V1.2.1  |  RAT         |   Multiple bug fixes.   No output to motors-fixed   Output smashed on one side of bot but not the other-fixed
*                 |              |      Speed/sensitivity button not working-fixed
*         V1.3    |  RAT         |   Added structure for autonomous code to be written for many conditions
*         V1.4    |  RAT         |   Added hardwere configuration file to the project to aid in abillity to rapidly settup the program for a
*                 |              |      robot built with any of our hardware and formatted the whitespace and removed extra comments to aid
*                 |              |      in legability
*         V1.5    | RAT          |   Added abillity to swap left and right motors in HardwareConfig.h
*         v1.6    | RAT          |   Added button opperated pnumatics configuration
*         V1.7    | RAT          |   One autonomous program has been added and runs as expected. No testing of the actual path has been performed.
*
*         !!!!!!!!!!UPDATE VERSION HISTORY BEFORE COMMIT!!!!!!!!!!
*    !!!!!!!!!!UPDATE VERSION HISTORY BEFORE COMMIT!!!!!!!!!!
*                  !!!!!!!!!!UPDATE VERSION HISTORY BEFORE COMMIT!!!!!!!!!!
*
*
*/





/*                         TO DO /  NOTES
*    1. Add selectable functions for autonomus mode
*                    need to put the structure for autonomous mode initialization code based on the chosen user input
*                red is opposite from blue        
*       a. Left start position Task "A"
*       b. Center start position Task "A"
*       c. Right start position Task "A"
*       d. Left start position Task "B"
*       e. Center start position Task "B"
*       f. Right start position Task "B"
*    2. Figure out how to take user input from the driver station
*    3. Add a "transmition selection" button to limmit speed of the bot
*    4. Qualification matches 12 total w/ min of 2 between each
*    5. Task A: leave zone
*    6. Task B: place cone or cube
*    7. Task C: ballance on charging station
*    8. Pnumatics control to program structure
*    9. Figure out how to create timing without stopping entire program
*                       Potential solution is to use an incrimenting for loop
*                       as a crude timer. store the current incriment to a variable
*                       when we need a start time and compare agianst current
*                       incriment to get elapsed time.
*       a. What type of value is returned by the timer.h functions?
*   10. Test previous commit from other pc just to be sure it works. It would suck to put in all this effort assuming the basic program works.
*   11. Add code in disabled state to ensure all motors are stationary
*   12. Add preprosser directives to easily swap between motor controlers and to invert motors     (theres an inverted version of this function)
*   13. Add the abillity to swap between drive styles (arcade and tank)
*   14. Passing the true parameter to the tankdrive function adds slow speed sensitivity
*   15. 
*   16.
*   17.
*   18.
*   19.
*   20.
*   21.
*   22.
*   23.
*   24.
*   25.
*/

/*
*   Criteria:
*   code for 4 pnumatics configurations:
*     *single action timed
*     *single action while button pressed
*     *double action timed
*     *double action while button pressed
*   settup options in HardwareConfig.h
*
*
*/


#include "Robot.h"
#include "HardwareConfig.h"
#include "AutonomousConfig.h"
#include "TimerMillis.h"
#include <frc/GenericHID.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>
//#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <rev/CANSparkMax.h>


using std::cout;
using std::endl;


#ifdef LOGITECH_F310
  const int               leftStick =  1;
  const int              rightStick =  5;
  const int     leftStickHorizontal =  0;
  const int              leftBumper =  5;
  const int             rightBumper =  6;
  const int                 aButton =  1; 
  const int                 yButton =  4;
#endif
bool                leftBumperPos =  false;
bool               rightBumperPos =  false;
bool                    bumperPos =  false;
bool                   aButtonPos =  false;
bool                   yButtonPos =  false;
bool               lastAButtonPos =  false;
bool               lastYButtonPos =  false;
bool              routineComplete =  true;
float                leftStickPos =  0;
float               rightStickPos =  0;
float      leftStickHorizontalPos =  0;
float    leftStickHorizontalSpeed =  0;
float                 sensitivity = .6;
bool                highSpeedMode =  true;
bool                    driveMode =  true;
float                   leftSpeed =  0;
float                  rightSpeed =  0;
long                       timeMS =  0;
int                  swichCaseNum =  0;
/*
const int b1 = 7;
const int b2 = 8;
const int b3 = 9;
const int b4 = 10;
const int b5 = 11;
const int b6 = 12;
*/

std::string m_autoSelected;
int AutonomousSelection(std::string chooserVal) {
  //std::cout << "chooserVal DBG: " << chooserVal << endl;
  if (chooserVal ==   "Red Left start position")     return 0;
  if (chooserVal == "Red Center start position")     return 1;
  if (chooserVal ==  "Red Right start position")     return 2;

  if (chooserVal ==   "Blue Left start position")    return 3;
  if (chooserVal == "Blue Center start position")    return 4;
  if (chooserVal ==  "Blue Right start position")    return 5;
  return 0;
}


//><><><><><><><><><><><><><><><><><><>  Functions  <><><><><><><><><><><><><><><><><><><

/* 
namespace AutonomusTaskSelect {
  enum task {
    leftStartRed,
    centerStartRed,
    rightStartRed,

    leftStartBlue,
    centerStartBlue,
    rightStartBlue,
  };
  
} // namespace AutonomusTaskSelect

AutonomusTaskSelect::task AutonomusSelection(std::string m_autoSelected) {
  if (m_autoSelected ==   "leftStartRed")     return AutonomusTaskSelect::task::    leftStartRed;
  if (m_autoSelected == "centerStartRed")     return AutonomusTaskSelect::task::  centerStartRed;
  if (m_autoSelected ==  "rightStartRed")     return AutonomusTaskSelect::task::   rightStartRed;

  if (m_autoSelected ==   "leftStartBlue")    return AutonomusTaskSelect::task::    leftStartBlue;
  if (m_autoSelected == "centerStartBlue")    return AutonomusTaskSelect::task::  centerStartBlue;
  if (m_autoSelected ==  "rightStartBlue")    return AutonomusTaskSelect::task::   rightStartBlue;
  
  return AutonomusTaskSelect::task::leftStartRed;
}
 */





//><><><><><><><><><><><><><><><><><><>  Functions  <><><><><><><><><><><><><><><><><><><


void Robot::RobotInit() {            // Code here will run once when enabled in any mode
  #ifdef INVERT_DRIVETRAIN
    rightMotors.SetInverted(true);
    leftMotors.SetInverted(false);
  #endif

  #ifndef INVERT_DRIVETRAIN
    rightMotors.SetInverted(false);
    leftMotors.SetInverted(true);
  #endif

  m_chooser.SetDefaultOption    (leftStartRed    , leftStartRed    );    //this id the default option if the user selects nothing
  m_chooser.AddOption           (centerStartRed  , centerStartRed  );    //these set the options available for the user to select
  m_chooser.AddOption           (rightStartRed   , rightStartRed   );

  m_chooser.AddOption           (leftStartBlue    , leftStartBlue  );
  m_chooser.AddOption           (centerStartBlue  , centerStartBlue);
  m_chooser.AddOption           (rightStartBlue   , rightStartBlue );
  frc::SmartDashboard::PutData ("Auto Modes" , &m_chooser );



  //frc::PneumaticHub::MakeDoubleSolenoid coneLauncher( 1 , 2 );
  //coneLauncher.Set(frc::DoubleSolenoid::Value::kReverse);
  
}

void Robot::RobotPeriodic() {        // Code here will run once every 20ms
  timeMS += 20;   // incriment time by 20 ms
}
#ifdef AUTO_DAVE_MODE
typedef struct 
{
   double LeftSpeed;
   double RightSpeed;
   int    DurationmS;  
} AutoStep;

typedef struct
{
    int NumSteps;
    AutoStep *pSteps;
} AutoProgramList;

//
// Auto program step/duration information.
//

//TODO: Add new AutoStep arrays, one for each auto program you want to be able to run.

// Trivial Auto program - drive forward at 50% speed for 1 second then drive backwards at
// 50% speed for another second.
AutoStep g_SimpleAutoStepList[] =
{
   { 0.5, 0.5,   1000},
   { -0.5, -0.5, 1000},
};  
int g_NumSimpleAutoSteps = sizeof(g_SimpleAutoStepList) / sizeof(AutoStep);

// TODO: Add your next auto program here...

// TODO: Add new step lists here, one per auto program. Add an entry in g_AutoProgramList for each one you add.
// The chooser needs to be set up so that the value chosen maps to the index into this array.
AutoProgramList g_AutoProgramList[] =
{
    { g_NumSimpleAutoSteps, g_SimpleAutoStepList }, // Index 0
    // TODO: Add a new entry for each auto program defined above here.
};

int g_NumAutoPrograms = sizeof(g_AutoProgramList) / sizeof(AutoProgramList);
#endif

void Robot::AutonomousInit() {       // Code here will run once upon recieving the command to enter autonomous mode
  #ifdef AUTO_SWICH_CASE
    robotDriveTrain.TankDrive(0.0, 0.0);
    timeMS = 0;
    m_autoSelected = m_chooser.GetSelected();
    swichCaseNum = AutonomousSelection(m_autoSelected);
    std::cout << "Auto mode selected:  " << m_autoSelected << endl;
    //std::cout << "Swich case num:  " << swichCaseNum << endl;
    #endif
  #ifdef AUTO_DAVE_MODE
  // TODO: m_chooser.GetSelected() returns the label string. Replace this with the method
  // that returns the value associated with the entry or the entry index.
  //AutoProgramIndex = m_chooser.GetSelected();

  std::cout << "Auto mode selected:  " << m_autoSelected << endl;

  // TODO: Use the value from the chooser to select one of the auto program step lists
  // you defined above. I suggest you create another array containing pointers to each
  // independent auto program's list and make the chooser value an index into this 
  // array. For now, this hardcodes the single example created above.
  AutoProgramIndex = 0;

  if(AutoProgramIndex >= g_NumAutoPrograms)
  {
    // TODO: This is bad - your chooser contains more entries than you have 
    // auto programs defined. Fix that!
    bAutoDisabled = true;
    return;
  }
  #endif
  //frc::Timer autonomousTimer.Stop();
  //frc::Timer autonomousTimer.Reset();
  #ifdef AUTO_SWICH_CASE
  switch (swichCaseNum) {
  //switch (m_autoSelected) {
  case 0: //leftStartRed
    //Robot::DriveForward(300, 1);
    break;

  case 1: //centerStartRed:
    //code for selected option goes here
    break;

  case 2: //rightStartRed:
  std::cout << "right start a init" << endl;
/*     Robot::DriveForward(750, -1);
    Robot::DriveStop(100);
    Robot::TurnLeftQuarter();
    Robot::DriveStop(100);
    Robot::DriveForward(2000, 1);
    Robot::DriveStop(100);
    Robot::TurnLeftQuarter();
    Robot::DriveStop(100);
    Robot::DriveForward(5000, 1);
    Robot::DriveStop(100);

     */

    break;




  case 3: //leftStartBlue:
    //code for selected option goes here
    break;

  case 4: //centerStartBlue:
    //code for selected option goes here
    break;

  case 5: //rightStartBlue:
    //code for selected option goes here
    break;
}
#endif
#ifdef AUTO_DAVE_MODE
  // Start the first step of the chosen auto program.
  AutoTimer = 0;
  AutoStep  = 0;

  // Start the whole auto program buy programming the motor speeds for the first step.
  robotDriveTrain.TankDrive(g_AutoProgramList[AutoProgramIndex].pSteps[AutoStep].LeftSpeed,
                            g_AutoProgramList[AutoProgramIndex].pSteps[AutoStep].RightSpeed);
#endif
}

void Robot::AutonomousPeriodic() {   // Code here will run right after RobotPeriodic() if the command is sent for autonomous mode0
#ifdef AUTO_DAVE_MODE
  if(bAutoDisabled)
  {
    robotDriveTrain.TankDrive(0.0, 0.0);
    return;
  }

  // Add 20mS to our step time.
  AutoTimer += 20;

  int StepTimeout = g_AutoProgramList[AutoProgramIndex].pSteps[AutoStep].DurationmS;

  if(AutoTimer >= StepTimeout)
  {
    // Move to the next step in the list.
    AutoStep++;

    // Have we reached the end of the program?
    if(AutoStep >=  g_AutoProgramList[AutoProgramIndex].NumSteps)
    {
      // We're finished this program.
      robotDriveTrain.TankDrive(0.0, 0.0);
      bAutoDisabled = true;
      return;
    }

    // We now need to set the motor speeds for the next step and zero the timer so
    // that we can count up to the next step limit.
    robotDriveTrain.TankDrive(g_AutoProgramList[AutoProgramIndex].pSteps[AutoStep].LeftSpeed,
                              g_AutoProgramList[AutoProgramIndex].pSteps[AutoStep].RightSpeed);
    AutoTimer = 0;
  }
  #endif

  #ifdef AUTO_SWICH_CASE
  switch (swichCaseNum) {
  case 0: //leftStartRed:
    robotDriveTrain.TankDrive(0.3, 0.3);
    break;

  case 1: //centerStartRed:
    //code for selected option goes here
    break;

  case 2: //rightStartRed:
    if(timeMS < 750) {                             // back 750
      robotDriveTrain.TankDrive(-1, -1);
    } else if(timeMS > 750 && timeMS < 850) {      // stop 100
      robotDriveTrain.TankDrive(0, 0);
    } else if(timeMS > 850 && timeMS < 1250) {     // turn 400
      robotDriveTrain.TankDrive(.6, -.6);
    } else if(timeMS > 1250 && timeMS < 1350) {    // stop 100
      robotDriveTrain.TankDrive(0, 0);
    } else if(timeMS > 1350 && timeMS < 3350) {    // forward 2000
      robotDriveTrain.TankDrive(1, 1);
    } else if(timeMS > 3350 && timeMS < 3450) {    // stop 100
      robotDriveTrain.TankDrive(0, 0);
    } else if(timeMS > 3450 && timeMS < 3850) {    // turn 400
      robotDriveTrain.TankDrive(.6, -.6);
    } else if(timeMS > 3850 && timeMS < 3950) {
      robotDriveTrain.TankDrive(0, 0);
    } else if(timeMS > 3950 && timeMS < 8950) {
      robotDriveTrain.TankDrive(1, 1);
    } else if (timeMS > 8950) {
      robotDriveTrain.TankDrive(0, 0);
    }
    break;

  case 3: //leftStartBlue:
    //code for selected option goes here
    break;

  case 4: //centerStartBlue:
    //code for selected option goes here
    break;

  case 5: //rightStartBlue:
    //code for selected option goes here
    break;
  }
  #endif
}


void Robot::TeleopInit() {           // Code here will run once upon recieving the command to enter autonomous mode
  robotDriveTrain.TankDrive(0.0, 0.0);
}

void Robot::TeleopPeriodic() {       // Code here will run right after RobotPeriodic() if the command is sent for manual control mode
    //units::time::second_t timeNow = frc::GetTime();

    #ifdef LOGITECH_F310
      leftStickPos           =    f310.GetRawAxis(          leftStick);   //gets joystick position and updates variable
      rightStickPos          =    f310.GetRawAxis(         rightStick);
      leftStickHorizontalPos =    f310.GetRawAxis(leftStickHorizontal);
      leftBumperPos          =  f310.GetRawButton(         leftBumper);
      rightBumperPos         =  f310.GetRawButton(        rightBumper);
      aButtonPos             =  f310.GetRawButton(            aButton);
      yButtonPos             =  f310.GetRawButton(            yButton);
    #endif

    if (leftBumperPos == true || rightBumperPos == true) {
      bumperPos = true;
    } else {
      bumperPos = false;
    }

    if (aButtonPos != lastAButtonPos) {   // Toggles between high and low sensitivity driving mode
      if (aButtonPos) {
        highSpeedMode = !highSpeedMode;
      }
      lastAButtonPos = aButtonPos;
    }

    if (yButtonPos != lastYButtonPos) {   // Toggles between tank and arcade drive
      if (yButtonPos) {
        driveMode = !driveMode;
      }
      lastYButtonPos = yButtonPos;
    }


    if (driveMode) {
      if (highSpeedMode) {           // Default mode is high sensitivity. If high speed mode is true, it wont apply any limmit to output
        leftSpeed  =  leftStickPos;
        rightSpeed = rightStickPos;
        robotDriveTrain.TankDrive(leftSpeed, rightSpeed);
      } else {                       // If high speed mode is false, it will multiply the stick position by sensitivity.
        leftSpeed  =  leftStickPos * sensitivity;
        rightSpeed = rightStickPos * sensitivity;
        robotDriveTrain.TankDrive(leftSpeed, rightSpeed);
      }
    } else {
      if (highSpeedMode) {           // Default mode is high sensitivity. If high speed mode is true, it wont apply any limmit to output
        leftSpeed  =  leftStickPos;
        robotDriveTrain.ArcadeDrive(leftSpeed, leftStickHorizontalPos);
      } else {                       // If high speed mode is false, it will multiply the stick position by sensitivity.
        leftSpeed  =  leftStickPos * sensitivity;
        leftStickHorizontalSpeed = leftStickHorizontalPos * sensitivity;
        robotDriveTrain.ArcadeDrive(leftSpeed, leftStickHorizontalSpeed);
      }
    }

    //std::cout << "before" << endl;
    #ifndef PNEUMATICS_HUB
    if (bumperPos) {
      //std::cout << "Bumper" << endl;
      //routineComplete = Robot::DriveForward(1000, .5);
    
       //Robot::DriveForward(5000, .5);
       //std::cout << "Drive" << endl;
    
      
    }
    #endif
    
    #ifdef PNEUMATICS_HUB
    if (bumperPos) {
      coneLauncher.Set(frc::DoubleSolenoid::Value::kForward);
      //set pnumatics to extended position
    } else {
      coneLauncher.Set(frc::DoubleSolenoid::Value::kReverse);
      //set pnumatics to retracted position
    }
    #endif

    //std::cout << leftStickPos << "         " << rightStickPos << endl; // prints the speed value to the terminal for troubleshooting
    //std::cout <<  "b1     " <<f310.GetRawButton(1) << "               b2    " << f310.GetRawButton(2) << "               b3    " << f310.GetRawButton(3) << "               b4    " << f310.GetRawButton(4) << "               b5    " << f310.GetRawButton(5) << "               b6    " << f310.GetRawButton(6) <<  endl;
 // }
}



void Robot::DisabledInit() {         // Code here will run once upon recieving the command to enter disabled mode
  robotDriveTrain.TankDrive(0.0, 0.0);
}

void Robot::DisabledPeriodic() {     // Code here will run right after RobotPeriodic() if the command is sent for autonomous mode
  robotDriveTrain.TankDrive(0.0, 0.0);
}



void Robot::TestInit() {             // Code here will run once upon recieving the command to enter test mode
}

void Robot::TestPeriodic() {         // Code here will run right after RobotPeriodic() if the command is sent for test mode
}



void Robot::SimulationInit() {       // Code here will run once upon recieving the command to enter autonomous mode
}

void Robot::SimulationPeriodic() {   // Code here will run right after RobotPeriodic() if the command is sent for simulation mode
}



#ifndef RUNNING_FRC_TESTS
int main() {                        // THIS IS ALWAYSE RUNNING    DONT EDIT THIS CODE
  return frc::StartRobot<Robot>();
}
#endif
 