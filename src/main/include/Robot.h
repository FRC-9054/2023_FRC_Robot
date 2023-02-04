

#pragma once

#include <frc/TimedRobot.h>
#include <frc/GenericHID.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Timer.h>
#include <iostream>
#include <string>




class Robot : public frc::TimedRobot {

  ctre::phoenix::motorcontrol::can::WPI_VictorSPX  leftMotor1 {0};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX  leftMotor2 {1};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX rightMotor1 {2};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX rightMotor2 {3};

  frc::MotorControllerGroup  leftMotors {  leftMotor1, leftMotor2  };
  frc::MotorControllerGroup rightMotors { rightMotor1, rightMotor2 };

  frc::Timer timeNowAutonomous;       //timer object created for timing based decisions in autonomous mode
  frc::Timer timeNowTeleop;           //timer object created for timing based decisions in teleop mode

  frc::DifferentialDrive robotDriveTrain {leftMotors, rightMotors};
  frc::GenericHID f310 {0};



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
  const std::string leftStartA = "Left start position Task A";
  const std::string centerStartA = "Center start position Task A";
  const std::string rightStartA = "Right start position Task A";
  const std::string leftStartB = "Left start position Task B";
  const std::string centerStartB = "Center start position Task B";
  const std::string rightStartB = "Right start position Task B";
  

private:
  
};
