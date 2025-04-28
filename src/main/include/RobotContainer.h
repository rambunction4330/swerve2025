// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <string>
#include <unordered_map>
#include "frc/smartdashboard/SendableChooser.h"
#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/ElevatorSubsystem.h"
#include "Telemetry.h"

class RobotContainer {
private:
    units::meters_per_second_t MaxSpeed = TunerConstants::kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
    units::radians_per_second_t MaxAngularRate = 0.75_tps; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
        .WithDeadband(MaxSpeed * 0.1).WithRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage); // Use open-loop control for drive motors
    swerve::requests::SwerveDriveBrake brake{};
    swerve::requests::PointWheelsAt point{};

    /* Note: This must be constructed before the drivetrain, otherwise we need to
     *       define a destructor to un-register the telemetry from the drivetrain */
    Telemetry logger{MaxSpeed};

    frc2::CommandXboxController joystick{0};

    rmb::LogitechGamepad elevatorController{1};

    std::unordered_map<std::string, frc2::CommandPtr> autoCommands; 

    frc::SendableChooser<std::string> autoChooser; 
    

public:
    subsystems::CommandSwerveDrivetrain drivetrain{TunerConstants::CreateDrivetrain()};

    RobotContainer();

    void loadAutos(); 

    frc2::CommandPtr autoDriveCommand(); 

    frc2::CommandPtr GetAutonomousCommand();

    void runAutoCommand();

    ElevatorSubsystem elevatorSubsystem;

private:
    void ConfigureBindings();
};
