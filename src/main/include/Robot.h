

#pragma once

#include "HardwareConfig.h"
#include <frc/TimedRobot.h>
#include <frc/GenericHID.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Timer.h>
#include <frc/DoubleSolenoid.h>
#include <rev/CANSparkMax.h>
#include <iostream>
#include <string>




class Robot : public frc::TimedRobot {

  #ifdef VICTOR_SPX_CAN
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX  leftMotor1 {0};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX  leftMotor2 {1};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX rightMotor1 {2};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX rightMotor2 {3};
  #endif

  #ifdef SPARKMAX_CAN
  rev::CANSparkMax  leftMotor1 {1 , rev::CANSparkMaxLowLevel::MotorType::kBrushed};    //swap 1 and 2 with 3 and 4
  rev::CANSparkMax  leftMotor2 {2 , rev::CANSparkMaxLowLevel::MotorType::kBrushed};
  rev::CANSparkMax rightMotor1 {3 , rev::CANSparkMaxLowLevel::MotorType::kBrushed};
  rev::CANSparkMax rightMotor2 {4 , rev::CANSparkMaxLowLevel::MotorType::kBrushed};
  #endif


  #ifdef SWAP_LEFT_AND_RIGHT
    frc::MotorControllerGroup rightMotors {  leftMotor1, leftMotor2  };
    frc::MotorControllerGroup  leftMotors { rightMotor1, rightMotor2 };
  #endif

  #ifndef SWAP_LEFT_AND_RIGHT
    frc::MotorControllerGroup  leftMotors {  leftMotor1, leftMotor2  };
    frc::MotorControllerGroup rightMotors { rightMotor1, rightMotor2 };
  #endif

  frc::Timer timeNowAutonomous;       //timer object created for timing based decisions in autonomous mode
  frc::Timer timeNowTeleop;           //timer object created for timing based decisions in teleop mode

  frc::DifferentialDrive robotDriveTrain {leftMotors, rightMotors};
  frc::GenericHID f310 {0};

  #ifdef PNEUMATICS_HUB
    frc::DoubleSolenoid coneLauncher{6, frc::PneumaticsModuleType::REVPH, 4, 5};
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
  const std::string leftStartARed = "Red Left start position Task A";
  const std::string centerStartARed = "Red Center start position Task A";
  const std::string rightStartARed = "Red Right start position Task A";
  const std::string leftStartBRed = "Red Left start position Task B";
  const std::string centerStartBRed = "Red Center start position Task B";
  const std::string rightStartBRed = "Red Right start position Task B";
  const std::string leftStartCRed = "Red Left start position Task C";
  const std::string centerStartCRed = "Red Center start position Task C";
  const std::string rightStartCRed = "Red Right start position Task C";

  const std::string leftStartABlue = "Blue Left start position Task A";
  const std::string centerStartABlue = "Blue Center start position Task A";
  const std::string rightStartABlue = "Blue Right start position Task A";
  const std::string leftStartBBlue = "Blue Left start position Task B";
  const std::string centerStartBBlue = "Blue Center start position Task B";
  const std::string rightStartBBlue = "Blue Right start position Task B";
  const std::string leftStartCBlue = "Blue Left start position Task C";
  const std::string centerStartCBlue = "Blue Center start position Task C";
  const std::string rightStartCBlue = "Blue Right start position Task C";
  

private:
  
};
