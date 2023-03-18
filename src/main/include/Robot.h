

#pragma once

#include "HardwareConfig.h"
#include "AutonomousConfig.h"
#include "TimerMillis.h"
#include <frc/TimedRobot.h>
#include <frc/GenericHID.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include <frc/DoubleSolenoid.h>
#include <rev/CANSparkMax.h>
#include <iostream>
#include <string>
#include <frc/RobotState.h>
#include <frc/BuiltInAccelerometer.h>


using std::cout;
using std::endl;



class Robot : public frc::TimedRobot {

  #ifdef VICTOR_SPX_CAN
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX  leftMotor1 {0};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX  leftMotor2 {1};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX rightMotor1 {2};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX rightMotor2 {3};
  #endif

  #ifdef SPARKMAX_CAN
  rev::CANSparkMax  leftMotor1 {1 , rev::CANSparkMaxLowLevel::MotorType::kBrushed};
  rev::CANSparkMax  leftMotor2 {2 , rev::CANSparkMaxLowLevel::MotorType::kBrushed};
  rev::CANSparkMax rightMotor1 {3 , rev::CANSparkMaxLowLevel::MotorType::kBrushed};
  rev::CANSparkMax rightMotor2 {4 , rev::CANSparkMaxLowLevel::MotorType::kBrushed};
  #endif


  #ifdef SWAP_LEFT_AND_RIGHT
    frc::MotorControllerGroup rightMotors {  leftMotor1, leftMotor2  };
    frc::MotorControllerGroup  leftMotors { rightMotor1, rightMotor2 };
  #else
    frc::MotorControllerGroup  leftMotors {  leftMotor1, leftMotor2  };
    frc::MotorControllerGroup rightMotors { rightMotor1, rightMotor2 };
  #endif


  frc::DifferentialDrive robotDriveTrain {leftMotors, rightMotors};
  #ifdef LOGITECH_F310
  frc::GenericHID f310 {1};
  #endif
  #ifdef EXTREME_3D_PRO
  frc::GenericHID extreme3D {0};
  #endif

  #ifdef PNEUMATICS_HUB
    frc::DoubleSolenoid coneLauncher{6, frc::PneumaticsModuleType::REVPH, 0, 1};
  #endif

  #ifdef BALANCE
    frc::BuiltInAccelerometer rioAccel{};
  #endif

#ifdef AUTO_SWICH_CASE
frc::Timer m_timer;
frc::SmartDashboard::PutNumber ("start delay", 0.0);
#endif

  #ifdef AUTO_DAVE_MODE
  private:
    int AutoTimer        = 0;
    int AutoStep         = 0;
    int AutoProgramIndex = 0;
    bool bAutoDisabled   = false;
  #endif

 public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;


  frc::SendableChooser<std::string> m_chooser;
  const std::string leftStartRed = "Left Red";
  const std::string centerStartRed = "Center Red";
  const std::string rightStartRed = "Right Red";

  const std::string leftStartBlue = "Left Blue";
  const std::string centerStartBlue = "Center Blue";
  const std::string rightStartBlue = "Right blue";
  const std::string TestCrawl = "Test Crawl";
  /*
uint64_t timeeee;
  void DriveForward(uint64_t timeToDrive, double speedToDrive) {
    //std::cout << "DriveForward()" << std::endl;
    TimerMillis tempTimer;
    tempTimer.Stop();
    tempTimer.Reset();
    tempTimer.Start();
    bool isRunning = true;
    while (isRunning) {
      timeeee = tempTimer.Get();
      //std::cout << timeeee << endl;
      if (timeeee <= timeToDrive) {
        robotDriveTrain.TankDrive(speedToDrive, speedToDrive);
        //std::cout << "     false" << endl;
      } else {
        isRunning = false;
        tempTimer.~TimerMillis();
        //std::cout << "     true" << endl;
      }
    }
    //return true;
  }

  void DriveStop(uint64_t stopTime) {
    TimerMillis tempTimer;
    tempTimer.Stop();
    tempTimer.Reset();
    tempTimer.Start();
    bool isRunning = true;
    while (isRunning) {
      if (tempTimer.Get() <= stopTime) {
        robotDriveTrain.TankDrive(0.0, 0.0);
        //return false;
      } else {
        isRunning = false;
        tempTimer.~TimerMillis();
        //return true;
      }
    }
  }

  void TurnLeftQuarter() {
    TimerMillis tempTimer;
    uint64_t stopTime = 400; 
    tempTimer.Stop();
    tempTimer.Reset();
    tempTimer.Start();
    bool isRunning = true;
    while (isRunning) {
      if (tempTimer.Get() <= stopTime) {
        robotDriveTrain.TankDrive(0.6, -0.6);
        //return false;
      } else {
        isRunning = false;
        tempTimer.~TimerMillis();
        //return true;
      }
    }
  }

  void TurnRightQuarter() {
    TimerMillis tempTimer;
    uint64_t stopTime = 400; 
    tempTimer.Stop();
    tempTimer.Reset();
    tempTimer.Start();
    bool isRunning = true;
    while (isRunning) {
      if (tempTimer.Get() <= stopTime) {
        robotDriveTrain.TankDrive(-0.6, 0.6);
        //return false;
      } else {
        isRunning = false;
        tempTimer.~TimerMillis();
        //return true;
      }
    }
  }

  void TurnLeftHalf() {
    TimerMillis tempTimer;
    uint64_t stopTime = 1000; 
    tempTimer.Stop();
    tempTimer.Reset();
    tempTimer.Start();
    bool isRunning = true;
    while (isRunning) {
      if (tempTimer.Get() <= stopTime) {
        robotDriveTrain.TankDrive(0.6, -0.6);
        //return false;
      } else {
        isRunning = false;
        tempTimer.~TimerMillis();
        //return true;
      }
    }
  }

  void TurnrightHalf() {
    TimerMillis tempTimer;
    uint64_t stopTime = 1000; 
    tempTimer.Stop();
    tempTimer.Reset();
    tempTimer.Start();
    bool isRunning = true;
    while (isRunning) {
      if (tempTimer.Get() <= stopTime) {
        robotDriveTrain.TankDrive(-0.6, 0.6);
        //return false;
      } else {
        isRunning = false;
        tempTimer.~TimerMillis();
        //return true;
      }
    }
  }
  
*/
private:
//double currentSpeed;
//double targetSpeed;
//double targetTolerance;

/*
bool RampTo() {
  double rampVal = currentSpeed - targetSpeed;
  rampVal = abs(rampVal);
  while (rampVal <= targetTolerance) {
    
  }
  
}
  */
};
