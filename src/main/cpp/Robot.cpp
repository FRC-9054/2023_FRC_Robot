

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
#include <frc/GenericHID.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <rev/CANSparkMax.h>


using std::cout;
using std::endl;



const int               leftStick =  1;
const int              rightStick =  5;
const int              leftBumper =  5;
const int             rightBumper =  6;
const int                 aButton =  1; 
bool                leftBumperPos =  false;
bool               rightBumperPos =  false;
bool                    bumperPos =  false;
bool                   aButtonPos =  false;
bool               lastAButtonPos =  false;
float                leftStickPos =  0;
float               rightStickPos =  0;
float                 sensitivity = .6;
bool                highSpeedMode =  true;
float                   leftSpeed =  0;
float                  rightSpeed =  0;

const int b1 = 7;
const int b2 = 8;
const int b3 = 9;
const int b4 = 10;
const int b5 = 11;
const int b6 = 12;

std::string m_autoSelected = "leftStartA";



//><><><><><><><><><><><><><><><><><><>  Functions  <><><><><><><><><><><><><><><><><><><\\


namespace AutonomusTaskSelect {
  enum task {
    leftStartARed,
    centerStartARed,
    rightStartARed,
    leftStartBRed,
    centerStartBRed,
    rightStartBRed,
    leftStartCRed,
    centerStartCRed,
    rightStartCRed,

    leftStartABlue,
    centerStartABlue,
    rightStartABlue,
    leftStartBBlue,
    centerStartBBlue,
    rightStartBBlue,
    leftStartCBlue,
    centerStartCBlue,
    rightStartCBlue,
  };
  
} // namespace AutonomusTaskSelect

AutonomusTaskSelect::task AutonomusSelection(std::string m_autoSelected) {
  if (m_autoSelected ==   "leftStartARed")     return AutonomusTaskSelect::task::    leftStartARed;
  if (m_autoSelected == "centerStartARed")     return AutonomusTaskSelect::task::  centerStartARed;
  if (m_autoSelected ==  "rightStartARed")     return AutonomusTaskSelect::task::   rightStartARed;
  if (m_autoSelected ==   "leftStartBRed")     return AutonomusTaskSelect::task::    leftStartBRed;
  if (m_autoSelected == "centerStartBRed")     return AutonomusTaskSelect::task::  centerStartBRed;
  if (m_autoSelected ==  "rightStartBRed")     return AutonomusTaskSelect::task::   rightStartBRed;
  if (m_autoSelected ==   "leftStartCRed")     return AutonomusTaskSelect::task::    leftStartCRed;
  if (m_autoSelected == "centerStartCRed")     return AutonomusTaskSelect::task::  centerStartCRed;
  if (m_autoSelected ==  "rightStartCRed")     return AutonomusTaskSelect::task::   rightStartCRed;

  if (m_autoSelected ==   "leftStartABlue")    return AutonomusTaskSelect::task::    leftStartABlue;
  if (m_autoSelected == "centerStartABlue")    return AutonomusTaskSelect::task::  centerStartABlue;
  if (m_autoSelected ==  "rightStartABlue")    return AutonomusTaskSelect::task::   rightStartABlue;
  if (m_autoSelected ==   "leftStartBBlue")    return AutonomusTaskSelect::task::    leftStartBBlue;
  if (m_autoSelected == "centerStartBBlue")    return AutonomusTaskSelect::task::  centerStartBBlue;
  if (m_autoSelected ==  "rightStartBBlue")    return AutonomusTaskSelect::task::   rightStartBBlue;
  if (m_autoSelected ==   "leftStartCBlue")    return AutonomusTaskSelect::task::    leftStartCBlue;
  if (m_autoSelected == "centerStartCBlue")    return AutonomusTaskSelect::task::  centerStartCBlue;
  if (m_autoSelected ==  "rightStartCBlue")    return AutonomusTaskSelect::task::   rightStartCBlue;
  return AutonomusTaskSelect::task::leftStartARed;
}



//><><><><><><><><><><><><><><><><><><>  Functions  <><><><><><><><><><><><><><><><><><><\\


void Robot::RobotInit() {            // Code here will run once when enabled in any mode
  #ifdef INVERT_DRIVETRAIN
    rightMotors.SetInverted(true);
    leftMotors.SetInverted(false);
  #endif

  #ifndef INVERT_DRIVETRAIN
    rightMotors.SetInverted(false);
    leftMotors.SetInverted(true);
  #endif

  m_chooser.SetDefaultOption    (leftStartARed    , leftStartARed    );    //this id the default option if the user selects nothing
  m_chooser.AddOption           (centerStartARed  , centerStartARed  );    //these set the options available for the user to select
  m_chooser.AddOption           (rightStartARed   , rightStartARed   );
  m_chooser.AddOption           (leftStartBRed    , leftStartBRed    );
  m_chooser.AddOption           (centerStartBRed  , centerStartBRed  );
  m_chooser.AddOption           (rightStartBRed   , rightStartBRed   ); 
  m_chooser.AddOption           (leftStartCRed    , leftStartCRed    );
  m_chooser.AddOption           (centerStartCRed  , centerStartCRed  );
  m_chooser.AddOption           (rightStartCRed   , rightStartCRed   );

  m_chooser.AddOption           (leftStartABlue   , leftStartABlue   ); 
  m_chooser.AddOption           (centerStartABlue , centerStartABlue ); 
  m_chooser.AddOption           (rightStartABlue  , rightStartABlue  );
  m_chooser.AddOption           (leftStartBBlue   , leftStartBBlue   );
  m_chooser.AddOption           (centerStartBBlue , centerStartBBlue );
  m_chooser.AddOption           (rightStartBBlue  , rightStartBBlue  ); 
  m_chooser.AddOption           (leftStartCBlue   , leftStartCBlue   );
  m_chooser.AddOption           (centerStartCBlue , centerStartCBlue );
  m_chooser.AddOption           (rightStartCBlue  , rightStartCBlue  );
  frc::SmartDashboard::PutData  ("Auto Modes"     , &m_chooser       );
  //frc::PneumaticHub::MakeDoubleSolenoid coneLauncher( 1 , 2 );
  //coneLauncher.Set(frc::DoubleSolenoid::Value::kReverse);
  
}

void Robot::RobotPeriodic() {        // Code here will run once every 50ms

}



void Robot::AutonomousInit() {       // Code here will run once upon recieving the command to enter autonomous mode
  m_autoSelected = m_chooser.GetSelected();

  std::cout << "Auto mode selected:  " << m_autoSelected << endl;



  switch (AutonomusSelection(m_autoSelected)) {
  case AutonomusTaskSelect::task::leftStartARed:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::centerStartARed:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::rightStartARed:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::leftStartBRed:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::centerStartBRed:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::rightStartBRed:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::leftStartCRed:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::centerStartCRed:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::rightStartCRed:
    //code for selected option goes here
    break;




  case AutonomusTaskSelect::task::leftStartABlue:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::centerStartABlue:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::rightStartABlue:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::leftStartBBlue:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::centerStartBBlue:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::rightStartBBlue:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::leftStartCBlue:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::centerStartCBlue:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::rightStartCBlue:
    //code for selected option goes here
    break;
  
  default:
    //run default code if no selection is made
    break;
  }
}

void Robot::AutonomousPeriodic() {   // Code here will run right after RobotPeriodic() if the command is sent for autonomous mode



  switch (AutonomusSelection(m_autoSelected)) {
  case AutonomusTaskSelect::task::leftStartARed:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::centerStartARed:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::rightStartARed:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::leftStartBRed:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::centerStartBRed:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::rightStartBRed:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::leftStartCRed:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::centerStartCRed:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::rightStartCRed:
    //code for selected option goes here
    break;




  case AutonomusTaskSelect::task::leftStartABlue:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::centerStartABlue:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::rightStartABlue:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::leftStartBBlue:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::centerStartBBlue:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::rightStartBBlue:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::leftStartCBlue:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::centerStartCBlue:
    //code for selected option goes here
    break;

  case AutonomusTaskSelect::task::rightStartCBlue:
    //code for selected option goes here
    break;
  
  default:
    //run default code if no selection is made
    break;
  }

}



void Robot::TeleopInit() {           // Code here will run once upon recieving the command to enter autonomous mode

}

void Robot::TeleopPeriodic() {       // Code here will run right after RobotPeriodic() if the command is sent for manual control mode
 // for (int currentTime = 1; currentTime > 0; currentTime++) {  //FOUND BETTER WAY // for use in timing applications       //       float timeNow = frc::GetTime();

    leftStickPos    =    f310.GetRawAxis(    leftStick);   //gets joystick position and updates variable
    rightStickPos   =    f310.GetRawAxis(   rightStick);
    leftBumperPos   =  f310.GetRawButton( leftBumper);
    rightBumperPos  =  f310.GetRawButton(rightBumper);
    aButtonPos      =  f310.GetRawButton(    aButton);

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


    if (highSpeedMode) {           // Default mode is high sensitivity. If high speed mode is true, it wont apply any limmit to output
      leftSpeed  =  leftStickPos;
      rightSpeed = rightStickPos;
      robotDriveTrain.TankDrive(leftSpeed, rightSpeed);
    } else {                       // If high speed mode is false, it will multiply the stick position by sensitivity.
      leftSpeed  =  leftStickPos * sensitivity;
      rightSpeed = rightStickPos * sensitivity;
      robotDriveTrain.TankDrive(leftSpeed, rightSpeed);
    }
    
    #ifdef PNEUMATICS_HUB
    if (bumperPos) {
      coneLauncher.Set(frc::DoubleSolenoid::Value::kForward);
      //set pnumatics to extended position
    } else {
      coneLauncher.Set(frc::DoubleSolenoid::Value::kReverse);
      //set pnumatics to retracted position
    }
    #endif PNEUMATICS_HUB

    //std::cout << leftStickPos << "         " << rightStickPos << endl; // prints the speed value to the terminal for troubleshooting
    //std::cout <<  "b1     " <<f310.GetRawButton(1) << "               b2    " << f310.GetRawButton(2) << "               b3    " << f310.GetRawButton(3) << "               b4    " << f310.GetRawButton(4) << "               b5    " << f310.GetRawButton(5) << "               b6    " << f310.GetRawButton(6) <<  endl;
 // }
}



void Robot::DisabledInit() {         // Code here will run once upon recieving the command to enter disabled mode
}

void Robot::DisabledPeriodic() {     // Code here will run right after RobotPeriodic() if the command is sent for autonomous mode
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
 