// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "frc/DriverStation.h"
#include "frc/Filesystem.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc2/command/Command.h"
#include "frc2/command/CommandPtr.h"
#include "pathplanner/lib/auto/AutoBuilder.h"
#include "pathplanner/lib/auto/NamedCommands.h"
#include "pathplanner/lib/commands/PathPlannerAuto.h"
#include "subsystems/ElevatorSubsystem.h"
#include <exception>
#include <filesystem>
#include <frc2/command/Commands.h>
#include <iostream>
#include <string>

RobotContainer::RobotContainer() {
  ConfigureBindings();
  loadAutos();
//   pathplanner::NamedCommands::registerCommand(
//       "set L3 pose", elevatorSubsystem.setElevatorStateCommand(12_cm, 0.31_tr));

//   pathplanner::NamedCommands::registerCommand(
//       "set L2 pose",
//       elevatorSubsystem.setElevatorStateCommand(7.5_cm, 0.31_tr));

//   pathplanner::NamedCommands::registerCommand(
//       "set rest pose", elevatorSubsystem.setElevatorStateCommand(0_cm, 0.1_tr));

//   pathplanner::NamedCommands::registerCommand(
//       "run intake out", elevatorSubsystem.runIntakeCommand());

  // pathplanner::NamedCommands::registerCommand("run intake out",
  // elevatorSubsystem.runIntake(const rmb::LogitechGamepad &conrtoller))
}

void RobotContainer::ConfigureBindings() {
  // Note that X is defined as forward according to WPILib convention,
  // and Y is defined as to the left according to WPILib convention.
  drivetrain.SetDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.ApplyRequest([this]() -> auto && {
        return drive
            .WithVelocityX(-joystick.GetLeftY() *
                           MaxSpeed) // Drive forward with negative Y (forward)
            .WithVelocityY(-joystick.GetLeftX() *
                           MaxSpeed) // Drive left with negative X (left)
            .WithRotationalRate(-joystick.GetRightX() *
                                MaxAngularRate); // Drive counterclockwise with
                                                 // negative X (left)
      }));
  elevatorSubsystem.SetDefaultCommand(
      elevatorSubsystem.teleopCommand(elevatorController));

  // NOT USEFUL DONT UNCOMMENT THESE
  // joystick.A().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& { return
  // brake; })); joystick.B().WhileTrue(drivetrain.ApplyRequest([this]() ->
  // auto&& {
  //     return point.WithModuleDirection(frc::Rotation2d{-joystick.GetLeftY(),
  //     -joystick.GetLeftX()});
  // }));
  // Run SysId routines when holding back/start and X/Y.
  // Note that each routine should be run exactly once in a single log.

  (joystick.Back() && joystick.Y())
      .WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
  (joystick.Back() && joystick.X())
      .WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
  (joystick.Start() && joystick.Y())
      .WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
  (joystick.Start() && joystick.X())
      .WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

  // reset the field-centric heading on left bumper press
  joystick.LeftBumper().OnTrue(
      drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));

  drivetrain.RegisterTelemetry(
      [this](auto const &state) { logger.Telemeterize(state); });
}

void RobotContainer::loadAutos() {
  std::string pathDir =
      frc::filesystem::GetDeployDirectory() + "/pathplanner/autos/";
  for (const auto &entry : std::filesystem::directory_iterator(pathDir)) {
    if (entry.is_regular_file() &&
        entry.path().extension().string() == ".auto") {
      autoCommands.insert(
          {entry.path().stem().string(),
           pathplanner::PathPlannerAuto(entry.path().stem().string()).ToPtr()});
    }
  }
  for (const auto &kv : autoCommands) {
    autoChooser.AddOption(kv.first, kv.first);
  }
  frc::SmartDashboard::PutData("Auto Choices", &autoChooser);
}

void RobotContainer::runAutoCommand() {
  if (!autoChooser.GetSelected().empty()) {
    try {
      autoCommands.at(autoChooser.GetSelected()).Schedule();
    } catch (const std::exception &_e) {
    }
  }
}

frc2::CommandPtr RobotContainer::autoDriveCommand() { 
     return drivetrain.ApplyRequest([this]() -> auto && {
        return drive
            .WithVelocityX(-0.4_mps) // Drive forward with negative Y (forward)
            .WithVelocityY(0.0_mps) // Drive left with negative X (left)
            .WithRotationalRate(0.0_tps); // Drive counterclockwise with
                                          // negative X (left)
      })
      .WithTimeout(4.0_s);
    
}
frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  //return runAutoCommand();
}
