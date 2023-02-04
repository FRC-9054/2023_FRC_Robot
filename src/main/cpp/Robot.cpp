

/*         !!!!!!!!!!UPDATE VERSION HISTORY BEFORE COMMIT!!!!!!!!!!
*            !!!!!!!!!!UPDATE VERSION HISTORY BEFORE COMMIT!!!!!!!!!!
*        !!!!!!!!!!UPDATE VERSION HISTORY BEFORE COMMIT!!!!!!!!!!
*
*        Version  |  Developer   |   Comments About Changes
*        _________|______________|_____________________________________________________________________________________________________
*         V1      |  RAT         |   Created the framework for the robot code 
*         V1.1    |  RAT         |   Added some code to accomodate a single action pnumatics directly controled by either bumper
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
*   11. 
*   12. 
*   13. 
*   14. 
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


#include "Robot.h"
#include <frc/GenericHID.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

using std::cout;
using std::endl;



const int               leftStick =  1;
const int              rightStick =  5;
const int              leftBumper =  5;
const int             rightBumper =  6;
int                 leftBumperPos =  false;
int                rightBumperPos =  false;
int                     bumperPos =  false;
float                leftStickPos =  0;
float               rightStickPos =  0;
float                 sensitivity = .6;
float                   leftSpeed =  0;
float                  rightSpeed =  0;
bool             invertDriveTrain =  false;    // change this if the robot goes backwards when you want it to go forward

const int b1 = 7;
const int b2 = 8;
const int b3 = 9;
const int b4 = 10;
const int b5 = 11;
const int b6 = 12;




void Robot::RobotInit() {            // Code here will run once when enabled in any mode
  if (invertDriveTrain) {     // this statement determines what side to invert
    rightMotors.SetInverted(true);
    leftMotors.SetInverted(false);

  } else {
    rightMotors.SetInverted(false);
    leftMotors.SetInverted(true);

  }

  m_chooser.SetDefaultOption    (leftStartA   , leftStartA   );    //this id the default option if the user selects nothing
  m_chooser.AddOption           (centerStartA , centerStartA );    //these set the options available for the user to select
  m_chooser.AddOption           (rightStartA  , rightStartA  );
  m_chooser.AddOption           (leftStartB   , leftStartB   );
  m_chooser.AddOption           (centerStartB , centerStartB );
  m_chooser.AddOption           (rightStartB  , rightStartB  );
  frc::SmartDashboard::PutData  ("Auto Modes" , &m_chooser   );

  
}

void Robot::RobotPeriodic() {        // Code here will run once every 50ms

}



void Robot::AutonomousInit() {       // Code here will run once upon recieving the command to enter autonomous mode
  std::string m_autoSelected = m_chooser.GetSelected();
  std::cout << "Auto mode selected:  " << m_autoSelected << endl;
  //need to put the structure for autonomous mode initialization code based on the chosen user input
}

void Robot::AutonomousPeriodic() {   // Code here will run right after RobotPeriodic() if the command is sent for autonomous mode

}



void Robot::TeleopInit() {           // Code here will run once upon recieving the command to enter autonomous mode

}

void Robot::TeleopPeriodic() {       // Code here will run right after RobotPeriodic() if the command is sent for manual control mode
  leftStickPos    = f310.GetRawAxis(    leftStick);   //gets joystick position and updates variable
  rightStickPos   = f310.GetRawAxis(   rightStick);
  leftBumperPos   = f310.GetRawButton( leftBumper);
  rightBumperPos  = f310.GetRawButton(rightBumper);

  if (leftBumperPos == 1 || rightBumperPos == 1)
  {
    bumperPos = 1;
  } else
  {
    bumperPos = 0;
  }

  leftSpeed  =  leftStickPos * sensitivity;     //calculates desired motor speed based on sensitivity and user input
  rightSpeed = rightStickPos * sensitivity;
  robotDriveTrain.TankDrive(leftSpeed, rightSpeed); // sets the speed to each motor
  if (bumperPos == 1)
  {
    //set pnumatics to extended position
  } else
  {
    //set pnumatics to retracted position
  }
  
   
  //std::cout << leftSpeed << "         " << rightSpeed << endl; // prints the speed value to the terminal for troubleshooting
  //std::cout <<  "b1     " <<f310.GetRawButton(1) << "               b2    " << f310.GetRawButton(2) << "               b3    " << f310.GetRawButton(3) << "               b4    " << f310.GetRawButton(4) << "               b5    " << f310.GetRawButton(5) << "               b6    " << f310.GetRawButton(6) <<  endl;

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
