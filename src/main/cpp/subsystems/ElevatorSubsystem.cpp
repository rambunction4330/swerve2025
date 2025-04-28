// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ElevatorSubsystem.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/FunctionalCommand.h"
#include "frc2/command/InstantCommand.h"
#include "frc2/command/RunCommand.h"
#include "rmb/controller/LogitechGamepad.h"
#include "units/angle.h"
#include <cmath>
#include <cstdlib>
#include <iostream>

ElevatorSubsystem::ElevatorSubsystem()
    : elevator1(constants::elevator::elevatorControllerCreateInfo1),
      elevator2(constants::elevator::elevatorControllerCreateInfo2),
      wrist(constants::wrist::wristPositionControllerCreateInfo),
      intake(constants::wrist::intakeVelocityController),
      climber1(constants::climber::climberVelocityController1),
      climber2(constants::climber::climberVelocityController2) {
  wrist.setEncoderPosition(0.0_tr);
  elevator2.follow(elevator1, false);
}
units::turn_t ElevatorSubsystem::getElevatorPosition() {
  // std::cout << "elevator 1 position" << elevator1.getPosition().value()
  //           << std::endl;

  // std::cout << "elevator 2 position" << elevator2.getPosition().value()
  //           << std::endl;
  return elevator1.getPosition();
  return elevator2.getPosition();
}

units::turn_t ElevatorSubsystem::getWristPosition() {
  // std::cout << "wrist position" <<
  // ((units::turn_t)wrist.getPosition()).value()
  //           << std::endl;
  // std::cout<<"target position" << ((units::turn_t)
  // wrist.getTargetPosition()).value()<<std::endl;
  return wrist.getPosition();
}

void ElevatorSubsystem::setWristPosition(units::turn_t position) {
  wrist.setPosition(
      position, constants::wrist::wrist_kG *
                    /*(units::turn_t)*/ std::sin(
                        (double)(units::turn_t)wrist.getPosition().value()));
}

void ElevatorSubsystem::setElevatorPostion(units::meter_t position) {
  elevator1.setPosition(position /
                        constants::elevator::linearToAngularGearRatio);

  // elevator1.setPosition(0.005_tr);
}

void ElevatorSubsystem::runIntake(const rmb::LogitechGamepad &controller) {
  double power;
  if (controller.GetRightTrigger() == 1) {
    power = -1.0;
  } else if (controller.GetLeftTrigger() == 1) {
    power = 1.0;
  } else {
    power = 0.0;
  }
  setIntakePower(power);
}

frc2::CommandPtr ElevatorSubsystem::runIntakeCommand() {
  return frc2::FunctionalCommand([]() {}, [&]() { setIntakePower(-1.0); },
                                 [](bool interupted) {}, [&]() { return true; })
      .ToPtr();
}

frc2::CommandPtr ElevatorSubsystem::runClimberCommand() {

  return frc2::FunctionalCommand([]() {}, [&]() { climber1.setPower(0.4); },
                                 [](bool interupted) {},
                                 [&]() { return false; })
      .ToPtr();
}

frc2::CommandPtr
ElevatorSubsystem::teleopCommand(const rmb::LogitechGamepad &controller) {
  return frc2::RunCommand(
             [&]() {
               runIntake(controller);
               if (abs(controller.GetLeftY()) < 0.05) {
                 climber1.setPower(0.0);
                 climber2.setPower(0.0);
               } else {
                 climber1.setPower(controller.GetLeftY() / 2.0);
                 climber2.setPower(controller.GetLeftY() / 2.0);
               }

               if (controller.GetA() == true) {
                 setElevatorPostion(8.2_cm);
                 setWristPosition(0.3_tr);
               } else if (controller.GetY() == true) {
                 setWristPosition(0.24_tr);
               } else if (controller.GetB() == true) {
                 setElevatorPostion(12_cm);
                 setWristPosition(0.31_tr);
               } else if (controller.GetX() == true) {
                 setElevatorPostion(16_cm);
                 setWristPosition(0.27_tr);
               } else if (controller.GetRightBumper()) {
                 setElevatorPostion(5_cm);
                 setWristPosition(0.22_tr);
               } else if (controller.GetLeftBumper()) {
                 setElevatorPostion(10_cm);
                 setWristPosition(0.22_tr);
               } else if (abs(controller.GetRightY()) > 0.05) {
                 setWristPosition(0.1_tr +
                                  (units::turn_t)controller.GetRightY() / 50);
               } else {
                 setElevatorPostion(0_cm);
                 setWristPosition(0.1_tr);
               }
             },
             {this})
      .ToPtr();
}

bool ElevatorSubsystem::atTarget() {
  return elevator1.atTarget() && wrist.atTarget();
}

frc2::CommandPtr
ElevatorSubsystem::setElevatorStateCommand(units::meter_t elevatorPosition,
                                           units::turn_t wristPosition) {
  return frc2::FunctionalCommand([]() {},
                                 [&]() {
                                   setElevatorPostion(elevatorPosition);
                                   setWristPosition(wristPosition);
                                 },
                                 [](bool interupted) {},
                                 [&]() { return atTarget(); }, {this})
      .ToPtr();
}
// This method will be called once per scheduler run
void ElevatorSubsystem::Periodic() {}
