// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc/XboxController.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/button/CommandXboxController.h"
#include "rmb/controller/LogitechGamepad.h"
#include "rmb/motorcontrol/Talon/TalonFXPositionController.h"
#include "subsystems/ElevatorConstants.h"
#include "rmb/motorcontrol/sparkmax/SparkMaxPositionController.h"
#include "rmb/motorcontrol/sparkmax/SparkMaxVelocityController.h"
#include "units/angle.h"
#include "units/length.h"
#include <frc2/command/SubsystemBase.h>
#include <memory>

class ElevatorSubsystem : public frc2::SubsystemBase {
 public:
  ElevatorSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  units::turn_t getElevatorPosition(); 

  units::turn_t getWristPosition(); 

  void setWristPosition(units::turn_t position);

  void setElevatorPostion(units::meter_t position);

  inline void setIntakePower(double power){intake.setPower(power);}

  void runIntake(const rmb::LogitechGamepad &conrtoller); 

  frc2::CommandPtr runIntakeCommand();

  frc2::CommandPtr runClimberCommand(); 

  frc2::CommandPtr teleopCommand(const rmb::LogitechGamepad &conrtoller); 

  frc2::CommandPtr setElevatorStateCommand(units::meter_t elevatorPosition, units::turn_t wristPosition); 

  bool atTarget(); 

  


 private:

rmb::TalonFXPositionController elevator1; 

rmb::TalonFXPositionController elevator2;

rmb::SparkMaxPositionController wrist; 

rmb::SparkMaxVelocityController intake;

rmb::SparkMaxVelocityController climber1;

rmb::SparkMaxVelocityController climber2;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
