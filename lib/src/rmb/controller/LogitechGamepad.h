#pragma once

#include "frc/event/BooleanEvent.h"
#include "frc/event/EventLoop.h"
#include "wpi/MathExtras.h"
#include <frc/GenericHID.h>
#include <frc2/command/button/Trigger.h>

#include <iostream>

namespace rmb {

class LogitechGamepad : public frc::GenericHID {
public:
  struct Axes {
    constexpr static int leftX = 0;
    constexpr static int leftY = 1;
    constexpr static int leftTrigger = 2;

    constexpr static int rightX = 4;
    constexpr static int rightY = 5;
    constexpr static int rightTrigger = 3;
  };

  struct Buttons {
    constexpr static int leftStick = 9;
    constexpr static int rightStick = 10;

    constexpr static int rightBumper = 6;
    constexpr static int leftBumper = 5;

    constexpr static int X = 3;
    constexpr static int Y = 4;
    constexpr static int A = 1;
    constexpr static int B = 2;

    constexpr static int backButton = 7;
    constexpr static int startButton = 8;
  };

  LogitechGamepad(int channel, double deadZone = 0.0,
                  bool squareOutputs = false)
      : frc::GenericHID(channel), deadZone(deadZone),
        squareOutputs(squareOutputs) {}

  double GetLeftX() const {
    double raw = -GetRawAxis(Axes::leftX);
    if (std::abs(raw) < deadZone) {
      return 0.0;
    }

    raw = wpi::sgn(raw) * (std::abs(raw) - deadZone) /
          (1.0 - deadZone); // Normalize from range 0.0-1.0

    return squareOutputs ? std::copysign(raw * raw, raw) : raw;
  }
  frc::BooleanEvent LeftXLessThan(double threshold) const {
    return AxisLessThan(Axes::leftX, threshold, loop);
  }
  frc::BooleanEvent LeftXGreaterThan(double threshold) const {
    return AxisGreaterThan(Axes::leftX, threshold, loop);
  }

  double GetLeftY() const {
    double raw = GetRawAxis(Axes::leftY);
    if (std::abs(raw) < deadZone) {
      return 0.0;
    }

    raw = wpi::sgn(raw) * (std::abs(raw) - deadZone) /
          (1.0 - deadZone); // Normalize from range 0.0-1.0

    return squareOutputs ? std::copysign(raw * raw, raw) : raw;
  }

  frc::BooleanEvent LeftYLessThan(double threshold) const {
    return AxisLessThan(Axes::leftY, threshold, loop);
  }
  frc::BooleanEvent LeftYgreaterThan(double threshold) const {
    return AxisGreaterThan(Axes::leftY, threshold, loop);
  }

  bool GetLeftStickButton() const { return GetRawButton(Buttons::leftStick); }
  bool GetLeftStickButtonPressed() {
    return GetRawButtonPressed(Buttons::leftStick);
  }
  bool GetLeftStickButtonReleased() {
    return GetRawButtonPressed(Buttons::leftStick);
  }
  frc::BooleanEvent LeftStickButton() const { return Button(Buttons::leftStick, loop); }

  double GetRightX() const {
    double raw = -GetRawAxis(Axes::rightX);
    if (std::abs(raw) < deadZone) {
      return 0.0;
    }

    raw = wpi::sgn(raw) * (std::abs(raw) - deadZone) /
          (1.0 - deadZone); // Normalize from range 0.0-1.0

    return squareOutputs ? std::copysign(raw * raw, raw) : raw;
  }

  frc::BooleanEvent RightXLessThan(double threshold) const {
    return AxisLessThan(Axes::rightX, threshold, loop);
  }
  frc::BooleanEvent RightXgreaterThan(double threshold) const {
    return AxisGreaterThan(Axes::rightX, threshold, loop);
  }

  double GetRightY() const {
    double raw = GetRawAxis(Axes::rightY);
    if (std::abs(raw) < deadZone) {
      return 0.0;
    }

    raw = wpi::sgn(raw) * (std::abs(raw) - deadZone) /
          (1.0 - deadZone); // Normalize from range 0.0-1.0

    return squareOutputs ? std::copysign(raw * raw, raw) : raw;
  }

  frc::BooleanEvent RightYLessThan(double threshold) const {
    return AxisLessThan(Axes::rightY, threshold, loop);
  }
  frc::BooleanEvent RightYGearterThan(double threshold) const {
    return AxisGreaterThan(Axes::rightY, threshold, loop);
  }

  bool GetRightStickButton() const { return GetRawButton(Buttons::rightStick); }
  bool GetRightStickButtonPressed() {
    return GetRawButtonPressed(Buttons::rightStick);
  }
  bool GetRightStickButtonReleased() {
    return GetRawButtonPressed(Buttons::rightStick);
  }
  frc::BooleanEvent RightStickButton() const { return Button(Buttons::rightStick, loop); }

  double GetLeftTrigger() const {
    double raw = GetRawAxis(Axes::leftTrigger);
    if (std::abs(raw) < deadZone) {
      return 0.0;
    }

    raw = wpi::sgn(raw) * (std::abs(raw) - deadZone) /
          (1.0 - deadZone); // Normalize from range 0.0-1.0

    return squareOutputs ? std::copysign(raw * raw, raw) : raw;
  }
  frc::BooleanEvent LeftTriggerLessThan(double threshold) const {
    return AxisLessThan(Axes::leftTrigger, threshold, loop);
  }
  frc::BooleanEvent LeftTriggergreaterThan(double threshold) const {
    return AxisGreaterThan(Axes::leftTrigger, threshold, loop);
  }

  double GetRightTrigger() const {
    double raw = GetRawAxis(Axes::rightTrigger);
    if (std::abs(raw) < deadZone) {
      return 0.0;
    }

    raw = wpi::sgn(raw) * (std::abs(raw) - deadZone) /
          (1.0 - deadZone); // Normalize from range 0.0-1.0

    return squareOutputs ? std::copysign(raw * raw, raw) : raw;
  }
  frc::BooleanEvent RightTriggerLessThan(double threshold) const {
    return AxisLessThan(Axes::rightTrigger, threshold, loop);
  }
  frc::BooleanEvent RightTriggerGreaterThan(double threshold) const {
    return AxisGreaterThan(Axes::rightTrigger, threshold, loop);
  }

  bool GetLeftBumper() const { return GetRawButton(Buttons::leftBumper); }
  bool GetLeftBumperPressed() {
    return GetRawButtonPressed(Buttons::leftBumper);
  }
  bool GetLeftBumperReleased() {
    return GetRawButtonPressed(Buttons::leftBumper);
  }
  frc::BooleanEvent LeftBumper() const { return Button(Buttons::leftBumper, loop); }

  bool GetRightBumper() const { return GetRawButton(Buttons::rightBumper); }
  bool GetRightBumperPressed() {
    return GetRawButtonPressed(Buttons::rightBumper);
  }
  bool GetRightBumperReleased() {
    return GetRawButtonPressed(Buttons::rightBumper);
  }
  frc::BooleanEvent RightBumper() const { return Button(Buttons::rightBumper, loop); }

  bool GetX() const { return GetRawButton(Buttons::X); }
  bool GetXPressed() { return GetRawButtonPressed(Buttons::X); }
  bool GetXReleased() { return GetRawButtonReleased(Buttons::X); }
  frc::BooleanEvent X() const { return Button(Buttons::X, loop); }

  bool GetY() const { return GetRawButton(Buttons::Y); }
  bool GetYPressed() { return GetRawButtonPressed(Buttons::Y); }
  bool GetYReleased() { return GetRawButtonReleased(Buttons::Y); }
  frc::BooleanEvent Y() const { return Button(Buttons::Y, loop); }

  bool GetA() const { return GetRawButton(Buttons::A); }
  bool GetAPressed() { return GetRawButtonPressed(Buttons::A); }
  bool GetAReleased() { return GetRawButtonReleased(Buttons::A); }
  frc::BooleanEvent A() const { return Button(Buttons::A, loop); }

  bool GetB() const { return GetRawButton(Buttons::B); }
  bool GetBPressed() { return GetRawButtonPressed(Buttons::B); }
  bool GetBReleased() { return GetRawButtonReleased(Buttons::B); }
  frc::BooleanEvent B() const { return Button(Buttons::B, loop); }

  bool GetBackButton() const { return GetRawButton(Buttons::backButton); }
  bool GetBackButtonPressed() {
    return GetRawButtonPressed(Buttons::backButton);
  }
  bool GetBackButtonReleased() {
    return GetRawButtonReleased(Buttons::backButton);
  }
  frc::BooleanEvent BackButton() const { return Button(Buttons::backButton, loop); }

  bool GetStartButton() const { return GetRawButton(Buttons::startButton); }
  bool GetStartButtonPressed() {
    return GetRawButtonPressed(Buttons::startButton);
  }
  bool GetStartButtonReleased() {
    return GetRawButtonReleased(Buttons::startButton);
  }
  frc::BooleanEvent StartButton() const { return Button(Buttons::startButton, loop); }

private:
  frc::EventLoop * loop; 
  double deadZone;
  bool squareOutputs;
};
} // namespace rmb
