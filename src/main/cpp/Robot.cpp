

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
*         V1.5    |  RAT         |   Added abillity to swap left and right motors in HardwareConfig.h
*         v1.6    |  RAT         |   Added button opperated pnumatics configuration
*         V1.7    |  RAT         |   One autonomous program has been added and runs as expected. No testing of the actual path has been performed.
*         V1.8    |  RAT         |   Auto code structure redone. Very effective. Needs instructions for each field position. Now accepts a delay
*                 |              |      input from shuffelboard that is added to the start of the program. Also changed the sensitivity function
*                 |              |      to be an adjustable gradiant using the left and right triggers so that its more intuitive and adjustable.
*                 |              |      Support for the EXTREME 3D joystich was also added
*         V1.9    |  RAT         |   Left and right triggers are now propperly changing the sensitivity. Added drive mode and sensitivity
*                 |              |      indicators to the shuffelboard. They need to be configured so that it makes more sense visually. The
*                 |              |      drive mode toggle has been split into the x button (arcade) and the b button (tank).
*
*         !!!!!!!!!!UPDATE VERSION HISTORY BEFORE COMMIT!!!!!!!!!!
*    !!!!!!!!!!UPDATE VERSION HISTORY BEFORE COMMIT!!!!!!!!!!
*                  !!!!!!!!!!UPDATE VERSION HISTORY BEFORE COMMIT!!!!!!!!!!
*
*
*/





/*                         TO DO /  NOTES
*    1. Qualification matches 12 total w/ min of 2 between each
*    2. Add balance while button pressed feature
*    3. Sepperate out the drive mode buttons into 2 buttons
*    4. Sepperate out the speed function into 2 buttons
*    5. Make speed incrimentable and print out the current value if it has changed          //   axis 2 is slow     axis 3 is fast
*    6. Add 90 deg and 180 deg turn to Dpad
*    7. Add test code for hardware
*          1.) Left motors on forward then backward one at a time
*          2.) Right motors on forward then backward one at a time
*          3.) Readout of the pressure swich on the compressor to the dashboard to confirm functionality
*          4.) Pnumatics actuation after pressure swich says full
*          5.) Test of all buttons on controller being used     //  print out their value to the shuffelboard
*    8. Test turns for autonomous to be able to navigate in autonomous     // no good. way too inconsistant 
*    9. Command based structure change?
*   10. finish adding code updating progress of test code on shuffelboard, and make test code disable motor breaking.  make all other modes enable motor breaking
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
  const int                 bButton =  2;
  const int                 xButton =  3;
  const int                 yButton =  4;
  const int                lTrigger =  2;
  const int                rTrigger =  3;
  const float    requiredTriggerVal = .4;
  float              sensitivitySet =  1;
#endif
#ifdef EXTREME_3D_PRO
  const int                   yAxis =  1;
  float                    yAxisPos =  0;
  const int                   xAxis =  0;
  float                    xAxisPos =  0;
  const int                 trigger =  1;
  const int                throttle =  3;
  float                 throttlePos =  0;
#endif
bool                leftBumperPos =  false;
bool               rightBumperPos =  false;
bool                    bumperPos =  false;
bool                   aButtonPos =  false;
bool                   yButtonPos =  false;
bool                   bButtonPos =  false;
bool                   xButtonPos =  false;
bool               lastAButtonPos =  false;
bool               lastYButtonPos =  false;
bool              routineComplete =  true;
float                leftStickPos =  0;
float               rightStickPos =  0;
float      leftStickHorizontalPos =  0;
float    leftStickHorizontalSpeed =  0;
float                 lTriggerPos =  0;
float                 rTriggerPos =  0;
float                 sensitivity = .6;
float        sensitivityIncriment = .02;
bool                highSpeedMode =  true;
bool                    driveMode =  true;
float                   leftSpeed =  0;
float                  rightSpeed =  0;
int                  swichCaseNum =  0;
long                  timeElapsed =  0;
long                       timeMS =  0;
long                    startTime =  0;
int                   autoStepNum =  0;
int                         delay =  0;
int                    *ptr_Delay =  &delay;
bool                        left1 =  false;
bool                        left2 =  false;
bool                       right1 =  false;
bool                       right2 =  false;
float                  rampIncNum;
float                         spd =  0;
bool                       rampUp =  false;
bool                     rampDown =  false;
bool                    motorTest =  false;
bool               pneumaticsTest =  false;
int                 delayTimeTest =  0;
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
  if (chooserVal ==    "Left Red")    return 0;
  if (chooserVal ==  "Center Red")    return 1;
  if (chooserVal ==   "Right Red")    return 2;

  if (chooserVal ==   "Left Blue")    return 3;
  if (chooserVal == "Center Blue")    return 4;
  if (chooserVal ==  "Right Blue")    return 5;

  if (chooserVal ==  "Test Crawl")    return 6;
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

float map(float input, float inA, float inb, float outA, float outB) {      // map function for easy conversion of input to output values
  float output = outA + ((outB - outA) / (inb -inA)) * (input - inA);
  return output;
}

#ifdef Auto_Step_List
bool runAuto(int autoProg) {
  
  if (stepNum > programSteps) {
    return false;
  }
  return true;
}
#endif
//long GetTimeElapsed() {
 // timeMS - 
//}

float RampVal(float currentVal, float targetVal, float rampIncriment) {
  if (currentVal != targetVal){            // do we need to ramp?
    if (currentVal > targetVal) {          // if we are ramping down
      currentVal -= rampIncriment;
      return currentVal;
    } else if (currentVal < targetVal) {   // if we are ramping up
      currentVal += rampIncriment;
      return currentVal;
    } else {
      std::cout << "!!! N.F.G. !!!" << endl;    // shouldnt ever get here
      return 0;
    }
  } else {          // we are already at the intended value
    return targetVal;
  }
}



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

  m_chooser.AddOption           (TestCrawl        , TestCrawl      );
  frc::SmartDashboard::PutData  ("Auto Modes" , &m_chooser );


  frc::SmartDashboard::PutNumber ("Start Delay Millis", 0);
  frc::SmartDashboard::PutNumber ("Sensitivity", sensitivity);
  frc::SmartDashboard::PutString ("Drive Mode" , "TANK");
  frc::SmartDashboard::PutBoolean ("TANK" , true);
  frc::SmartDashboard::PutBoolean ("ARCADE" , false);
  frc::SmartDashboard::PutNumber ("Crawl Speed (.5 to 1)", 0);
  frc::SmartDashboard::PutNumber ("Crawl Time (in milliseconds)", 0);
  frc::SmartDashboard::PutString ("Basket Position", "Retracted");
  



  //frc::PneumaticHub::MakeDoubleSolenoid coneLauncher( 1 , 2 );
  //coneLauncher.Set(frc::DoubleSolenoid::Value::kReverse);
  
}

void Robot::RobotPeriodic() {        // Code here will run once every 20ms
  //timeMS += 20;   // incriment time by 20 ms
}



#ifdef AUTO_DAVE_MODE
typedef struct 
{
   double                      LeftSpeed;
   double                     RightSpeed;
   int                        DurationmS;
   frc::DoubleSolenoid::Value   position;
} AutoStep;

typedef struct
{
    int NumSteps;
    AutoStep *pSteps;
} AutoProgramList;

//
// Auto program step/duration information.
//



//  CHANGE THE LEFT RED AND RIGHT RED PROGRAMS SO THAT THEY DRIVE OUT PASSED THE LINE BY 2 FEET THEN RETURN AT A NO SLIP SPEED
//  THEN CREATE PROGRAM THAT CHECKS INPUT SPEED AND DURATION FROM SHUFFELBOARD IN ORDER TO GET THE RIGHT SPEED AND DURATION TO GET ACROSS THE CHARGING STATION


AutoStep g_LeftRedStepList[] =
{
   {  0.0,  0.0, delay, frc::DoubleSolenoid::Value::kReverse  },        // wait the perscribed ammount of time before executing the auto code
   {  0.0,  0.0,  1500, frc::DoubleSolenoid::Value::kForward  },
   {  0.6,  0.6,  2000, frc::DoubleSolenoid::Value::kReverse  },
};  
int g_NumLeftRedSteps = sizeof(g_LeftRedStepList) / sizeof(AutoStep);


AutoStep g_CenterRedStepList[] =
{
   {  0.0,  0.0, delay, frc::DoubleSolenoid::Value::kReverse  },        // wait the perscribed ammount of time before executing the auto code
   {  0.0,  0.0,  1500, frc::DoubleSolenoid::Value::kForward  },
   {  0.6,  0.6,  2000, frc::DoubleSolenoid::Value::kReverse  },
};  
int g_CenterRedSteps = sizeof(g_CenterRedStepList) / sizeof(AutoStep);


AutoStep g_RightRedStepList[] =
{
   {  0.0,  0.0, delay, frc::DoubleSolenoid::Value::kReverse  },        // wait the perscribed ammount of time before executing the auto code
   {  0.0,  0.0,  1500, frc::DoubleSolenoid::Value::kForward  },
   {  0.6,  0.6,  2000, frc::DoubleSolenoid::Value::kReverse  },
};  
int g_NumRightRedSteps = sizeof(g_RightRedStepList) / sizeof(AutoStep);

//    END OF RED PROGRAMS        END OF RED PROGRAMS


////////////////////////////////  BLUE SHOULD BE A MIRROR OF RED  //////////////////////////////////////


//   START OF BLUE PROGRAMS    START OF BLUE PROGRAMS

AutoStep g_LeftBlueStepList[] =
{
   {  0.0,  0.0, delay, frc::DoubleSolenoid::Value::kReverse  },        // wait the perscribed ammount of time before executing the auto code
   {  0.0,  0.0,  1500, frc::DoubleSolenoid::Value::kForward  },
   {  0.6,  0.6,  2000, frc::DoubleSolenoid::Value::kReverse  },
};  
int g_NumLeftBlueSteps = sizeof(g_LeftBlueStepList) / sizeof(AutoStep);


AutoStep g_CenterBlueStepList[] =
{
   {  0.0,  0.0, delay, frc::DoubleSolenoid::Value::kReverse  },        // wait the perscribed ammount of time before executing the auto code
   {  0.0,  0.0,  1500, frc::DoubleSolenoid::Value::kForward  },
   {  0.6,  0.6,  2000, frc::DoubleSolenoid::Value::kReverse  },
};  
int g_NumCenterBlueSteps = sizeof(g_CenterBlueStepList) / sizeof(AutoStep);


AutoStep g_RightBlueStepList[] =
{
   {  0.0,  0.0, delay, frc::DoubleSolenoid::Value::kReverse  },        // wait the perscribed ammount of time before executing the auto code
   {  0.0,  0.0,  1500, frc::DoubleSolenoid::Value::kForward  },
   {  0.6,  0.6,  2000, frc::DoubleSolenoid::Value::kReverse  },
};  
int g_NumRightBlueSteps = sizeof(g_RightBlueStepList) / sizeof(AutoStep);




AutoStep g_TestCrawlStepList[] =
{
   {  0.0,  0.0, delay, frc::DoubleSolenoid::Value::kReverse  },        // wait the perscribed ammount of time before executing the auto code
   {  0.0,  0.0,  1500, frc::DoubleSolenoid::Value::kForward  },
   {  0.0,  0.0,  0000, frc::DoubleSolenoid::Value::kReverse  },
};  
int g_NumTestCrawlSteps = sizeof(g_TestCrawlStepList) / sizeof(AutoStep);


// TODO: Add new step lists here, one per auto program. Add an entry in g_AutoProgramList for each one you add.
// The chooser needs to be set up so that the value chosen maps to the index into this array.
AutoProgramList g_AutoProgramList[] =
{
    { g_NumLeftRedSteps, g_LeftRedStepList },        // Index 0
    { g_CenterRedSteps, g_CenterRedStepList },       // Index 1
    { g_NumRightRedSteps, g_RightRedStepList },      // Index 2
    
    { g_NumLeftBlueSteps, g_LeftBlueStepList },      // Index 3
    { g_NumCenterBlueSteps, g_CenterBlueStepList },  // Index 4
    { g_NumRightBlueSteps, g_RightBlueStepList },    // Index 5

    { g_NumTestCrawlSteps, g_TestCrawlStepList },    // Index 6
    // TODO: Add a new entry for each auto program defined above here.
};

int g_NumAutoPrograms = sizeof(g_AutoProgramList) / sizeof(AutoProgramList);
#endif

void Robot::AutonomousInit() {       // Code here will run once upon recieving the command to enter autonomous mode
  #ifdef AUTO_ARRAY
    autoStepNum =  0;           //reset timing
    timeMS = 0;
    timeElapsed = 0;

    switch (swichCaseNum) {
      case 0:
      int autoSteps [] = {

      }
      break;

      case 1:
      break;

      case 2:
      break;



      case 3:
      break;

      case 4:
      break;

      case 5:
      break;
    }
  #endif
  #ifdef AUTO_SWICH_CASE
    robotDriveTrain.TankDrive(0.0, 0.0);
    timeMS = 0;
    m_autoSelected = m_chooser.GetSelected();
    m_timer.Reset();
    m_timer.Start();
    //m_autoSelected = 0;
    swichCaseNum = AutonomousSelection(m_autoSelected);
    std::cout << "Auto mode selected:  " << m_autoSelected << endl;
    //std::cout << "Swich case num:  " << swichCaseNum << endl;
  #endif
  #ifdef AUTO_DAVE_MODE
  // TODO: m_chooser.GetSelected() returns the label string. Replace this with the method
  // that returns the value associated with the entry or the entry index.
  //AutoProgramIndex = m_chooser.GetSelected();
  bAutoDisabled = false;

  m_autoSelected = m_chooser.GetSelected();
  delay = frc::SmartDashboard::GetNumber ("Start Delay Millis", 0);
  float TestCrawlSpeed = frc::SmartDashboard::GetNumber ("Crawl Speed (.5 to 1)", 0);
  float TestCrawlTime = frc::SmartDashboard::GetNumber ("Crawl Time (in milliseconds)", 0);
  AutoProgramIndex = AutonomousSelection(m_autoSelected);
  std::cout << "Auto mode selected:  " << m_autoSelected    << endl;
  std::cout <<         "Auto delay:  " << delay             << endl;
  //std::cout <<    "NumAutoPrograms:  " << g_NumAutoPrograms << endl;
  g_AutoProgramList[AutoProgramIndex].pSteps[AutoStep].DurationmS = delay;
  for (int i = 0; i < g_NumAutoPrograms; i++)
  {
    g_AutoProgramList[i].pSteps[0].DurationmS = delay;
  }

  g_AutoProgramList[6].pSteps[2].LeftSpeed = TestCrawlSpeed;
  g_AutoProgramList[6].pSteps[2].RightSpeed = TestCrawlSpeed;
  g_AutoProgramList[6].pSteps[2].DurationmS = TestCrawlTime;
  

  // TODO: Use the value from the chooser to select one of the auto program step lists
  // you defined above. I suggest you create another array containing pointers to each
  // independent auto program's list and make the chooser value an index into this 
  // array. For now, this hardcodes the single example created above.
  // AutoProgramIndex = 0;

  if(AutoProgramIndex >= g_NumAutoPrograms)
  {
    // TODO: This is bad - your chooser contains more entries than you have 
    // auto programs defined. Fix that!
    //std::cout << "disabled set to true" << endl;
    bAutoDisabled = true;
    return;
  }
  #endif
  //frc::Timer autonomousTimer.Stop();
  //frc::Timer autonomousTimer.Reset();
  #ifdef AUTO_SWICH_CASE
  switch (swichCaseNum) {
  //switch (m_autoSelectedptr_D {
  case 0: //leftStartRed
    //Robot::DriveForward(300, 1);
    break;

  case 1: //centerStartRed:
    //code for selected option goes here
    break;

  case 2: //rightStartRed:
  //std::cout << "right start a init" << endl;
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
           coneLauncher.Set(g_AutoProgramList[AutoProgramIndex].pSteps[AutoStep].position);
  //std::cout << "Timer:   " << AutoTimer << endl;
  //std::cout << "Step:    " << AutoStep  << endl;
  //std::cout << bAutoDisabled << endl;
#endif
}

void Robot::AutonomousPeriodic() {   // Code here will run right after RobotPeriodic() if the command is sent for autonomous mode0
  #ifdef AUTO_SWICH_CASE
   int x = frc::SmartDashboard::GetNumber ("start delay", 0);
   units::unit_t<units::time::second, double , units::linear_scale> secondsX(x);
  #endif
#ifdef AUTO_DAVE_MODE
  if(bAutoDisabled)
  {
    robotDriveTrain.TankDrive(0.0, 0.0);
    coneLauncher.Set(frc::DoubleSolenoid::Value::kReverse);
    //std::cout << "disabled" << endl;
    return;
  }

  // Add 20mS to our step time.
  AutoTimer += 20;

  int StepTimeout = g_AutoProgramList[AutoProgramIndex].pSteps[AutoStep].DurationmS;
  std::cout << "StepTime:   " << g_AutoProgramList[AutoProgramIndex].pSteps[AutoStep].DurationmS << endl;

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
      std::cout << "program complete" << endl;
      return;
    }
    //std::cout << "update motors" << endl;
    // We now need to set the motor speeds for the next step and zero the timer so
    // that we can count up to the next step limit.
    robotDriveTrain.TankDrive(g_AutoProgramList[AutoProgramIndex].pSteps[AutoStep].LeftSpeed,
                              g_AutoProgramList[AutoProgramIndex].pSteps[AutoStep].RightSpeed);
             coneLauncher.Set(g_AutoProgramList[AutoProgramIndex].pSteps[AutoStep].position);
    AutoTimer = 0;
  }
  robotDriveTrain.TankDrive(g_AutoProgramList[AutoProgramIndex].pSteps[AutoStep].LeftSpeed,
                            g_AutoProgramList[AutoProgramIndex].pSteps[AutoStep].RightSpeed);
           coneLauncher.Set(g_AutoProgramList[AutoProgramIndex].pSteps[AutoStep].position);
  //std::cout << "Timer:   " << AutoTimer << endl;
  //std::cout << "Step:    " << AutoStep  << endl;
  #endif

  #ifdef AUTO_SWICH_CASE
  switch (swichCaseNum) {
  case 0: //leftStartRed:      //side start
    if(m_timer.Get()< secondsX) {
      robotDriveTrain.TankDrive(0, 0);
    } else if(m_timer.Get() < 0.5_s + secondsX) {
      robotDriveTrain.TankDrive(-.75, -.75);
    } else if (m_timer.Get()<1.55_s +secondsX) {
      robotDriveTrain.TankDrive(0, 0);
    }else if(m_timer.Get()<3.35_s + secondsX) {
      robotDriveTrain.TankDrive(.75, .75);
    }else if(m_timer.Get()<3.4_s + secondsX) {
      robotDriveTrain.TankDrive(0, 0);
    }
    else{
      robotDriveTrain.TankDrive(0, 0);
    }
    break;

  case 1: //centerStartRed:   //center start
    if(m_timer.Get()< secondsX) {
      robotDriveTrain.TankDrive(0, 0);
    } else if(m_timer.Get() < 0.5_s + secondsX) {    //.5
      robotDriveTrain.TankDrive(-.75, -.75);
    } else if (m_timer.Get()<0.25_s + secondsX) {     //.25
      robotDriveTrain.TankDrive(0, 0);
    }else{
      robotDriveTrain.TankDrive(0, 0);
    }
    break;

  case 2: //rightStartRed:
    if(timeMS < 750) {                             // back 750
      robotDriveTrain.TankDrive(-1, -1);
    } else if(timeMS > 750 && timeMS < 950) {      // stop 200
      robotDriveTrain.TankDrive(0, 0);
    } else if(timeMS > 950 && timeMS < 1450) {     // turn 500
      robotDriveTrain.TankDrive(.6, -.6);
    } else if(timeMS > 1450 && timeMS < 1550) {    // stop 100
      robotDriveTrain.TankDrive(0, 0);
    } else if(timeMS > 1550 && timeMS < 3550) {    // forward 2000
      robotDriveTrain.TankDrive(1, 1);
    } else if(timeMS > 3550 && timeMS < 3650) {    // stop 100
      robotDriveTrain.TankDrive(0, 0);
    } else if(timeMS > 3650 && timeMS < 4150) {    // turn 500
      robotDriveTrain.TankDrive(.6, -.6);
    } else if(timeMS > 4150 && timeMS < 4250) {    // stop 100
      robotDriveTrain.TankDrive(0, 0);
    } else if(timeMS > 4250 && timeMS < 5250) {    // forward 1000
      robotDriveTrain.TankDrive(1, 1);
    } else if (timeMS >= 5250) {                   // stop
      robotDriveTrain.TankDrive(0, 0);
    }
    break;

  case 3: //leftStartBlue:
  if (timeMS < 2000) {                             // 1750
    robotDriveTrain.TankDrive(-.75, -.75);
  } else if (timeMS > 2000 && timeMS < 3200) {     // 1500
    robotDriveTrain.TankDrive(.75, .75);
  } else if (timeMS >= 3200) {                     // END
    robotDriveTrain.TankDrive(0, 0);
  }
  break;

  case 4: //centerStartBlue:       cube jolt
    if (timeMS < 1700) {
    robotDriveTrain.TankDrive(-.75, -.75);
  } else if (timeMS > 1700 && timeMS < 2100) {      // stop 400
    robotDriveTrain.TankDrive(0, 0);
  } else if (timeMS > 2100 && timeMS < 4100) {      // drive out 2000
    robotDriveTrain.TankDrive(.75, .75);
  } else if (timeMS >= 4100) {
    robotDriveTrain.TankDrive(0, 0);
  }
  break;

  case 5: //rightStartBlue:
    //code for selected option goes here
    break;
  }
  #endif
}


void Robot::TeleopInit() {           // Code here will run once upon recieving the command to enter autonomous mode
  robotDriveTrain.TankDrive(0.0, 0.0);
  sensitivity = 1;
  sensitivitySet = frc::SmartDashboard::GetNumber ("Sensitivity", sensitivitySet);
  sensitivity = map(sensitivitySet, 0,  1, 0.5, 1);
}

void Robot::TeleopPeriodic() {       // Code here will run right after RobotPeriodic() if the command is sent for manual control mode
    //units::time::second_t timeNow = frc::GetTime();

    #ifdef LOGITECH_F310
      leftStickPos           =    f310.GetRawAxis(          leftStick);   //gets joystick position and updates variable
      rightStickPos          =    f310.GetRawAxis(         rightStick);
      leftStickHorizontalPos =    f310.GetRawAxis(leftStickHorizontal);
      lTriggerPos            =    f310.GetRawAxis(           lTrigger);
      rTriggerPos            =    f310.GetRawAxis(           rTrigger);
      leftBumperPos          =  f310.GetRawButton(         leftBumper);
      rightBumperPos         =  f310.GetRawButton(        rightBumper);
      aButtonPos             =  f310.GetRawButton(            aButton);
      yButtonPos             =  f310.GetRawButton(            yButton);
      bButtonPos             =  f310.GetRawButton(            bButton);
      xButtonPos             =  f310.GetRawButton(            xButton);
    #endif
    #ifdef EXTREME_3D_PRO
      yAxisPos               =    extreme3D.GetRawAxis(              yAxis);   //gets joystick position and updates variable
      throttlePos            =    extreme3D.GetRawAxis(           throttle);
      xAxisPos               =    extreme3D.GetRawAxis(              xAxis);
      leftBumperPos          =  extreme3D.GetRawButton(            trigger);
    #endif

    if (leftBumperPos == true || rightBumperPos == true) {
      bumperPos = true;
    } else {
      bumperPos = false;
    }
    #ifdef LOGITECH_F310
    /*
    if (aButtonPos != lastAButtonPos) {   // Toggles between high and low sensitivity driving mode
      if (aButtonPos) {
        highSpeedMode = !highSpeedMode;
      }
      lastAButtonPos = aButtonPos;
    }
    */
    //std::cout << "left:   " << lTriggerPos;
    //std::cout << "        right:   " << rTriggerPos << endl;
    if (lTriggerPos >= requiredTriggerVal && rTriggerPos >= requiredTriggerVal) {    // if both buttons are pressed ignore the inputs
      lTriggerPos = 0;
      rTriggerPos = 0;
      //std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
    } else if (lTriggerPos >= requiredTriggerVal && sensitivitySet > 0) {
      sensitivitySet -= sensitivityIncriment;
      sensitivity = map(sensitivitySet, 0,  1, 0.5, 1);
      frc::SmartDashboard::PutNumber ("Sensitivity", sensitivitySet);
      std::cout << "SensitivitySet:  " << sensitivitySet << "         Sensitivity:  " << endl;
      //std::cout << "down" << endl;
    } else if (rTriggerPos >= requiredTriggerVal && sensitivitySet < 1) {
      sensitivitySet += sensitivityIncriment;
      sensitivity = map(sensitivitySet, 0,  1, 0.5, 1);
      frc::SmartDashboard::PutNumber ("Sensitivity", sensitivitySet);
      std::cout << "SensitivitySet:  " << sensitivitySet << "         Sensitivity:  " << endl;
      //std::cout << "up" << endl;
    }
    //std::cout << "sensitivity set:   " << sensitivitySet << endl;

    //sensitivity = map(sensitivitySet, 0,  1, 0.5, 1);
    //std::cout << "sensitivity:   " << sensitivity << endl;

    /* if (yButtonPos != lastYButtonPos) {   // Toggles between tank and arcade drive
      if (yButtonPos) {
        driveMode = !driveMode;
      }
      lastYButtonPos = yButtonPos;
    } */


    if (xButtonPos == true && bButtonPos == true) {
      xButtonPos = false;
      bButtonPos = false;
    } else if (xButtonPos == true) {
      driveMode = true;
      frc::SmartDashboard::PutString ("Drive Mode" , "TANK");
      frc::SmartDashboard::PutBoolean ("TANK" , true);
      frc::SmartDashboard::PutBoolean ("ARCADE" , false);
    } else if (bButtonPos == true) {
      driveMode = false;
      frc::SmartDashboard::PutString ("Drive Mode" , "ARCADE");
      frc::SmartDashboard::PutBoolean ("TANK" , false);
      frc::SmartDashboard::PutBoolean ("ARCADE" , true);
    }


    if (driveMode) {
      leftSpeed  =  leftStickPos * sensitivity;
      rightSpeed = rightStickPos * sensitivity;
      robotDriveTrain.TankDrive(leftSpeed, rightSpeed);
    } else {
      leftSpeed  =  leftStickPos * sensitivity;
      leftStickHorizontalSpeed = leftStickHorizontalPos * sensitivity;
      robotDriveTrain.ArcadeDrive(leftSpeed, leftStickHorizontalSpeed);
    }

    
    #endif
    #ifdef EXTREME_3D_PRO
      sensitivity = map(throttlePos, 1, -1, 0.5, 1);
      yAxisPos    =    yAxisPos * sensitivity;
      xAxisPos    =    xAxisPos * sensitivity;
      robotDriveTrain.ArcadeDrive(yAxisPos, xAxisPos);
      //std::cout << sensitivity << endl;
    #endif
    
    #ifdef PNEUMATICS_HUB
    if (bumperPos) {
      coneLauncher.Set(frc::DoubleSolenoid::Value::kForward);
      frc::SmartDashboard::PutString ("Basket position" , "EXTENDED");
      //set pnumatics to extended position
    } else {
      coneLauncher.Set(frc::DoubleSolenoid::Value::kReverse);
      frc::SmartDashboard::PutString ("Basket position" , "RETRACTED");
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
  rampIncNum = .02;
  left1 = false;
  left2 = false;
  right1 = false;
  right2 = false;
  rampUp = false;
  rampDown = false;
  motorTest = false;
  delayTimeTest = 0;
  pneumaticsTest = false;
}

void Robot::TestPeriodic() {         // Code here will run right after RobotPeriodic() if the command is sent for test mode
    if (motorTest == false && pneumaticsTest == false) {
      if (left1 == false && left2 == false && right1 == false && right2 == false) {
        if (rampUp == false && rampDown == false) {
          if (spd != 1) {
            spd = RampVal(spd, 1, rampIncNum);
            leftMotor1.Set(spd);
            leftMotor2.Set(0.0);
            rightMotor1.Set(0.0);
            rightMotor2.Set(0.0);
            coneLauncher.Set(frc::DoubleSolenoid::Value::kReverse);
          } else {
            rampUp = true;
          }
        } else if (rampUp == true && rampDown == false) {
          if (spd != 1) {
            spd = RampVal(spd, -1, rampIncNum);
            leftMotor1.Set(spd);
            leftMotor2.Set(0.0);
            rightMotor1.Set(0.0);
            rightMotor2.Set(0.0);
            coneLauncher.Set(frc::DoubleSolenoid::Value::kReverse);
          } else {
            rampUp = true;
          }
        } else if (rampUp == true && rampDown == true) {
          rampUp = false;
          rampDown = false;
          left1 = true;
        }
      }

      if (left1 == true && left2 == false && right1 == false && right2 == false) {
        if (rampUp == false && rampDown == false) {
          if (spd != 1) {
            spd = RampVal(spd, 1, rampIncNum);
            leftMotor1.Set(0.0);
            leftMotor2.Set(spd);
            rightMotor1.Set(0.0);
            rightMotor2.Set(0.0);
            coneLauncher.Set(frc::DoubleSolenoid::Value::kReverse);
          } else {
            rampUp = true;
          }
        } else if (rampUp == true && rampDown == false) {
          if (spd != 1) {
            spd = RampVal(spd, -1, rampIncNum);
            leftMotor1.Set(0.0);
            leftMotor2.Set(spd);
            rightMotor1.Set(0.0);
            rightMotor2.Set(0.0);
            coneLauncher.Set(frc::DoubleSolenoid::Value::kReverse);
          } else {
            rampUp = true;
          }
        } else if (rampUp == true && rampDown == true) {
          rampUp = false;
          rampDown = false;
          left1 = true;
        }
      }

      if (left1 == true && left2 == true && right1 == false && right2 == false) {
        if (rampUp == false && rampDown == false) {
          if (spd != 1) {
            spd = RampVal(spd, 1, rampIncNum);
            leftMotor1.Set(0.0);
            leftMotor2.Set(0.0);
            rightMotor1.Set(spd);
            rightMotor2.Set(0.0);
            coneLauncher.Set(frc::DoubleSolenoid::Value::kReverse);
          } else {
            rampUp = true;
          }
        } else if (rampUp == true && rampDown == false) {
          if (spd != 1) {
            spd = RampVal(spd, -1, rampIncNum);
            leftMotor1.Set(0.0);
            leftMotor2.Set(0.0);
            rightMotor1.Set(spd);
            rightMotor2.Set(0.0);
            coneLauncher.Set(frc::DoubleSolenoid::Value::kReverse);
          } else {
            rampUp = true;
          }
        } else if (rampUp == true && rampDown == true) {
          rampUp = false;
          rampDown = false;
          left1 = true;
        }
      }

      if (left1 == true && left2 == true && right1 == true && right2 == false) {
        if (rampUp == false && rampDown == false) {
          if (spd != 1) {
            spd = RampVal(spd, 1, rampIncNum);
            leftMotor1.Set(0.0);
            leftMotor2.Set(0.0);
            rightMotor1.Set(0.0);
            rightMotor2.Set(spd);
            coneLauncher.Set(frc::DoubleSolenoid::Value::kReverse);
          } else {
            rampUp = true;
          }
        } else if (rampUp == true && rampDown == false) {
          if (spd != 1) {
            spd = RampVal(spd, -1, rampIncNum);
            leftMotor1.Set(0.0);
            leftMotor2.Set(0.0);
            rightMotor1.Set(0.0);
            rightMotor2.Set(spd);
            coneLauncher.Set(frc::DoubleSolenoid::Value::kReverse);
          } else {
            rampUp = true;
          }
        } else if (rampUp == true && rampDown == true) {
          rampUp = false;
          rampDown = false;
          left1 = true;
          motorTest = true;
        }
      }
    } else if (motorTest == true && pneumaticsTest == false) {
      if (delayTimeTest <= 1500) {
        coneLauncher.Set(frc::DoubleSolenoid::Value::kForward);
        robotDriveTrain.TankDrive(0.0, 0.0);
      } else {
        coneLauncher.Set(frc::DoubleSolenoid::Value::kReverse);
        robotDriveTrain.TankDrive(0.0, 0.0);
        pneumaticsTest = true;
      }
      delayTimeTest += 20;
    } else {
      coneLauncher.Set(frc::DoubleSolenoid::Value::kReverse);
        robotDriveTrain.TankDrive(0.0, 0.0);
    }
    frc::SmartDashboard::PutBoolean ("Left MTR 1" , left1);
    frc::SmartDashboard::PutBoolean ("Left MTR 2" , left2);
    frc::SmartDashboard::PutBoolean ("Right MTR 1" , right1);
    frc::SmartDashboard::PutBoolean ("Right MTR 2" , right2);
    frc::SmartDashboard::PutBoolean ("Mtr Forward" , false);
    frc::SmartDashboard::PutBoolean ("MTR Backward" , false);
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
 