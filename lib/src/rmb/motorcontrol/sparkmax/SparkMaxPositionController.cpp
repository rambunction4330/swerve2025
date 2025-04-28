#include "rmb/motorcontrol/sparkmax/SparkMaxPositionController.h"
#include "rev/ClosedLoopSlot.h"
#include "rev/SparkMax.h"
#include "rev/SparkRelativeEncoder.h"
#include "rev/config/ClosedLoopConfig.h"
#include "rev/config/LimitSwitchConfig.h"
#include "rev/config/SparkMaxConfig.h"

#include <algorithm>
#include <iostream>
#include <memory>
#include <ostream>
#include <utility>

namespace rmb {
SparkMaxPositionController::SparkMaxPositionController(
    const SparkMaxPositionController::CreateInfo &createInfo)
    : sparkMax(createInfo.motorConfig.id, createInfo.motorConfig.motorType),
      tolerance(createInfo.pidConfig.tolerance),
      feedforward(createInfo.feedforward),
      minPose(createInfo.range.minPosition),
      maxPose(createInfo.range.maxPosition),
      encoderType(createInfo.feedbackConfig.encoderType),
      gearRatio(createInfo.feedbackConfig.gearRatio) {

  rev::spark::SparkMaxConfig sparkConfig;
  sparkConfig.SmartCurrentLimit(
      static_cast<unsigned int>(createInfo.motorConfig.currentLimit() + 0.5));
  sparkConfig.Inverted(createInfo.motorConfig.inverted);
  // Restore defaults to ensure a consistent and clean slate.
  // sparkMax.RestoreFactoryDefaults();
  // sparkMax.SetSmartCurrentLimit(
  //     static_cast<unsigned int>(createInfo.motorConfig.currentLimit() +
  //     0.5));

  // Motor Configuration
  // sparkMax.SetInverted(createInfo.motorConfig.inverted);

  // PID Configuration
  sparkConfig.closedLoop.P(createInfo.pidConfig.p);
  sparkConfig.closedLoop.I(createInfo.pidConfig.i);
  sparkConfig.closedLoop.D(createInfo.pidConfig.d);
  sparkConfig.closedLoop.VelocityFF(createInfo.pidConfig.ff);
  sparkConfig.closedLoop.IZone(createInfo.pidConfig.iZone);
  sparkConfig.closedLoop.IMaxAccum(createInfo.pidConfig.iMaxAccumulator);
  sparkConfig.closedLoop.OutputRange(createInfo.pidConfig.minOutput,
                                     createInfo.pidConfig.maxOutput);

  // Range
  if (createInfo.range.isContinuous) {
    sparkConfig.closedLoop.PositionWrappingEnabled(true);
    sparkConfig.closedLoop.PositionWrappingMinInput(
        units::turn_t(createInfo.range.minPosition).to<double>() * gearRatio);
    sparkConfig.closedLoop.PositionWrappingMaxInput(
        units::turn_t(createInfo.range.maxPosition).to<double>() * gearRatio);
  }

  // Motion Profiling Configuration
  controlType = rev::spark::SparkMax::ControlType::kPosition;
  if (createInfo.profileConfig.useSmartMotion) {
    controlType = rev::spark::SparkMax::ControlType::kSmartMotion;
    sparkConfig.closedLoop.maxMotion.MaxVelocity(
        units::revolutions_per_minute_t(createInfo.profileConfig.maxVelocity)
            .to<double>() *
        gearRatio);
    sparkConfig.closedLoop.maxMotion.MaxAcceleration(
        units::revolutions_per_minute_per_second_t(
            createInfo.profileConfig.maxAcceleration)
            .to<double>() *
        gearRatio);
  }

  // Encoder Configuation

  switch (encoderType) {
  case EncoderType::HallSensor:
    encoder = std::make_unique<rev::spark::ClosedLoopConfig::FeedbackSensor>(
        sparkConfig.closedLoop.kPrimaryEncoder);
    // sparkConfig.encoder.CountsPerRevolution(
    //     createInfo.feedbackConfig.countPerRev);
    break;
  case EncoderType::Quadrature:
    encoder = std::make_unique<rev::spark::ClosedLoopConfig::FeedbackSensor>(
        sparkConfig.closedLoop.kPrimaryEncoder);

    // sparkConfig.encoder.CountsPerRevolution(
    //     createInfo.feedbackConfig.countPerRev);
    break;
  case EncoderType::Alternate:
    encoder = std::make_unique<rev::spark::ClosedLoopConfig::FeedbackSensor>(
        sparkConfig.closedLoop.kAlternateOrExternalEncoder);
    // sparkConfig.encoder.CountsPerRevolution(
    //     createInfo.feedbackConfig.countPerRev);
    break;

  case EncoderType::Absolute:
    encoder = std::make_unique<rev::spark::ClosedLoopConfig::FeedbackSensor>(
        sparkConfig.closedLoop.kAbsoluteEncoder);
    // sparkConfig.encoder.CountsPerRevolution(
    //     createInfo.feedbackConfig.countPerRev);
    break;
  }

  sparkConfig.closedLoop.SetFeedbackSensor(*encoder);

  // Limit Switch Configuaration

  switch (createInfo.feedbackConfig.forwardSwitch) {
  case LimitSwitchConfig::Disabled:
    sparkConfig.limitSwitch.ForwardLimitSwitchEnabled(false);

    break;
  case LimitSwitchConfig::NormalyOpen:
    sparkConfig.limitSwitch.ForwardLimitSwitchType(
        rev::spark::LimitSwitchConfig::kNormallyOpen);
    sparkConfig.limitSwitch.ForwardLimitSwitchEnabled(true);
    // sparkMax.GetForwardLimitSwitch(rev::spark::SparkLimitSwitchConfig::Type::kNormallyOpen)
    //     .EnableLimitSwitch(true);
    break;
  case LimitSwitchConfig::NormalyClosed:
    sparkConfig.limitSwitch.ForwardLimitSwitchType(
        rev::spark::LimitSwitchConfig::kNormallyClosed);
    sparkConfig.limitSwitch.ForwardLimitSwitchEnabled(true);
    // sparkMax.GetForwardLimitSwitch(rev::spark::SparkLimitSwitchConfig::Type::kNormallyClosed)
    //     .EnableLimitSwitch(true);
    break;
  }

  switch (createInfo.feedbackConfig.reverseSwitch) {
  case LimitSwitchConfig::Disabled:
    sparkConfig.limitSwitch.ReverseLimitSwitchEnabled(false);
    // sparkMax.GetReverseLimitSwitch(rev::SparkLimitSwitch::Type::kNormallyOpen)
    //     .EnableLimitSwitch(false);
    break;
  case LimitSwitchConfig::NormalyOpen:
    sparkConfig.limitSwitch.ReverseLimitSwitchType(
        rev::spark::LimitSwitchConfig::kNormallyOpen);
    sparkConfig.limitSwitch.ReverseLimitSwitchEnabled(true);
    // sparkMax.GetReverseLimitSwitch(rev::SparkLimitSwitch::Type::kNormallyOpen)
    //     .EnableLimitSwitch(true);
    break;
  case LimitSwitchConfig::NormalyClosed:
    sparkConfig.limitSwitch.ReverseLimitSwitchType(
        rev::spark::LimitSwitchConfig::kNormallyClosed);
    sparkConfig.limitSwitch.ReverseLimitSwitchEnabled(true);
    // sparkMax.GetReverseLimitSwitch(rev::SparkLimitSwitch::Type::kNormallyClosed)
    //     .EnableLimitSwitch(true);
    break;
  }

  // Follower Congiguration
  for (auto &follower : createInfo.followers) {

    rev::spark::SparkMaxConfig followerConfig;
    followers.emplace_back(std::make_unique<rev::spark::SparkMax>(
        follower.id, follower.motorType));
    followerConfig.Apply(sparkConfig).Follow(sparkMax);
    followers.back()->Configure(
        followerConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kPersistParameters);
    // followers.back()->Follow(sparkMax, follower.inverted);
  }

  sparkMax.Configure(sparkConfig,
                     rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                     rev::spark::SparkBase::PersistMode::kPersistParameters);
}

void SparkMaxPositionController::setPosition(units::radian_t position) {

  targetPosition =
      sparkMax.configAccessor.closedLoop.GetPositionWrappingEnabled()
          ? position
          : std::clamp(position, minPose, maxPose);
  sparkMax.GetClosedLoopController().SetReference(
      units::turn_t(targetPosition).to<double>() * gearRatio, controlType,
      rev::spark::ClosedLoopSlot::kSlot0,
      feedforward->calculateStatic(0.0_rpm, position).to<double>());
}

void SparkMaxPositionController::setPosition(units::radian_t position,
                                             double ff) {
  // std::cout << "setpoint a: " << ((units::turn_t)position).value() <<
  // std::endl;
  targetPosition =
      sparkMax.configAccessor.closedLoop.GetPositionWrappingEnabled()
          ? (units::turn_t)position
          : units::turn_t(std::clamp(((units::turn_t)position).value(),
                                     ((units::turn_t)minPose).value(),
                                     ((units::turn_t)maxPose).value()));

  // std::cout << "APPLIED FEEDFORWARD: " << ff << std::endl;
  sparkMax.GetClosedLoopController().SetReference(
      ((units::turn_t)targetPosition).value() * gearRatio, controlType,
      rev::spark::ClosedLoopSlot::kSlot0, ff, FeedforwardUnits);
  // std::cout << "setpoint b: " << ((units::turn_t)position).value() <<
  // std::endl;
}

units::radian_t SparkMaxPositionController::getTargetPosition() const {
  return targetPosition;
}

void SparkMaxPositionController::setPower(double power) {
  targetPosition = 0.0_rad;
  sparkMax.Set(power);
}

double SparkMaxPositionController::getPower() const { return sparkMax.Get(); }

units::radian_t SparkMaxPositionController::getMinPosition() const {
  return minPose;
}

units::radian_t SparkMaxPositionController::getMaxPosition() const {
  return maxPose;
}

void SparkMaxPositionController::disable() { sparkMax.Disable(); }

void SparkMaxPositionController::stop() { sparkMax.StopMotor(); }

units::radians_per_second_t SparkMaxPositionController::getVelocity() const {

  switch (encoderType) {
  case EncoderType::HallSensor:
  case EncoderType::Quadrature: {
    rev::spark::SparkRelativeEncoder *rel =
        static_cast<rev::spark::SparkRelativeEncoder *>(&sparkMax.GetEncoder());
    return units::revolutions_per_minute_t(rel->GetVelocity() / gearRatio);
    encoder.get();
  }
  case EncoderType::Alternate: {
    rev::spark::SparkMaxAlternateEncoder *alt =
        static_cast<rev::spark::SparkMaxAlternateEncoder *>(
            &sparkMax.GetAlternateEncoder());
    return units::revolutions_per_minute_t(alt->GetVelocity() / gearRatio);
  }
  case EncoderType::Absolute: {
    rev::AbsoluteEncoder *ab =
        static_cast<rev::AbsoluteEncoder *>(&sparkMax.GetAbsoluteEncoder());
    return units::revolutions_per_minute_t(ab->GetVelocity() / gearRatio);
  }
  }
  return 0_rpm;
}

units::radian_t SparkMaxPositionController::getPosition() const {

  switch (encoderType) {
  case EncoderType::HallSensor:
  case EncoderType::Quadrature: {
    rev::spark::SparkRelativeEncoder *rel =
        static_cast<rev::spark::SparkRelativeEncoder *>(&sparkMax.GetEncoder());
    return units::turn_t(rel->GetPosition() / gearRatio);
  }
  case EncoderType::Alternate: {
    rev::spark::SparkMaxAlternateEncoder *alt =
        static_cast<rev::spark::SparkMaxAlternateEncoder *>(
            &sparkMax.GetAlternateEncoder());
    return units::turn_t(alt->GetPosition() / gearRatio);
  }
  case EncoderType::Absolute: {
    rev::spark::SparkAbsoluteEncoder *ab =
        static_cast<rev::spark::SparkAbsoluteEncoder *>(
            &sparkMax.GetAbsoluteEncoder());
    return units::turn_t(ab->GetPosition() / gearRatio);
  }
  }
  return 0_rad;
}

void SparkMaxPositionController::setEncoderPosition(units::radian_t position) {
  switch (encoderType) {
  case EncoderType::HallSensor:
  case EncoderType::Quadrature: {
    rev::spark::SparkRelativeEncoder *rel =
        static_cast<rev::spark::SparkRelativeEncoder *>(&sparkMax.GetEncoder());
    rel->SetPosition(units::turn_t(position).to<double>() * gearRatio);
    break;
  }
  case EncoderType::Alternate: {
    rev::spark::SparkMaxAlternateEncoder *rel =
        static_cast<rev::spark::SparkMaxAlternateEncoder *>(
            &sparkMax.GetAlternateEncoder());
    rel->SetPosition(units::turn_t(position).to<double>() * gearRatio);
    break;
  }
  case EncoderType::Absolute: {
    rev::spark::SparkAbsoluteEncoder *ab =
        static_cast<rev::spark::SparkAbsoluteEncoder *>(
            &sparkMax.GetAbsoluteEncoder());
    // ab->SetZeroOffset(ab->GetPosition() +
    //                   units::turn_t(position).to<double>() / gearRatio);

    break;
  }
  }
}

units::radian_t SparkMaxPositionController::getTolerance() const {
  return tolerance;
}

} // namespace rmb
