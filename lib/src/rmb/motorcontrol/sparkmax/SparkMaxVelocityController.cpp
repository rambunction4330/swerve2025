#include "rmb/motorcontrol/sparkmax/SparkMaxVelocityController.h"
#include "rev/SparkAbsoluteEncoder.h"
#include "rev/SparkMaxAlternateEncoder.h"
#include "rev/config/ClosedLoopConfig.h"
#include "rev/config/SparkMaxConfig.h"

#include <units/angle.h>
#include <units/length.h>

namespace rmb {

SparkMaxVelocityController::SparkMaxVelocityController(
    const SparkMaxVelocityController::CreateInfo &createInfo)
    : sparkMax(createInfo.motorConfig.id, createInfo.motorConfig.motorType),
      tolerance(createInfo.pidConfig.tolerance),
      encoderType(createInfo.feedbackConfig.encoderType),
      gearRatio(createInfo.feedbackConfig.gearRatio) {

  // Restore defaults to ensure a consistent and clean slate.
  rev::spark::SparkMaxConfig sparkConfig;

  sparkConfig.SmartCurrentLimit(
      static_cast<unsigned int>(createInfo.motorConfig.currentLimit() + 0.5));
  sparkConfig.OpenLoopRampRate(createInfo.profileConfig.closedLoopRampRate());
  sparkConfig.ClosedLoopRampRate(createInfo.motorConfig.openLoopRampRate());

  // Motor Configuration
  sparkConfig.Inverted(createInfo.motorConfig.inverted);

  // PID Configuration
  sparkConfig.closedLoop.P(createInfo.pidConfig.p);
  sparkConfig.closedLoop.I(createInfo.pidConfig.i);
  sparkConfig.closedLoop.D(createInfo.pidConfig.d);
  sparkConfig.closedLoop.VelocityFF(createInfo.pidConfig.ff);
  sparkConfig.closedLoop.IZone(createInfo.pidConfig.iZone);
  sparkConfig.closedLoop.IMaxAccum(createInfo.pidConfig.iMaxAccumulator);
  sparkConfig.closedLoop.OutputRange(createInfo.pidConfig.minOutput,
                               createInfo.pidConfig.maxOutput);

  // Motion Profiling Configuration
  controlType = rev::spark::SparkMax::ControlType::kVelocity;
  if (createInfo.profileConfig.useSmartMotion) {
    controlType = rev::spark::SparkMax::ControlType::kSmartVelocity;
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
    // sparkConfig.encoder.CountsPerRevolution(createInfo.feedbackConfig.countPerRev);
    break;
  case EncoderType::Quadrature:
    encoder = std::make_unique<rev::spark::ClosedLoopConfig::FeedbackSensor>
    (sparkConfig.closedLoop.kPrimaryEncoder);
    // sparkConfig.encoder.CountsPerRevolution(createInfo.feedbackConfig.countPerRev);
    break;
  case EncoderType::Alternate:
    encoder = std::make_unique<rev::spark::ClosedLoopConfig::FeedbackSensor>
    (sparkConfig.closedLoop.kAlternateOrExternalEncoder);
    // sparkConfig.encoder.CountsPerRevolution(createInfo.feedbackConfig.countPerRev);
    break;
  case EncoderType::Absolute:
    encoder =
        std::make_unique<rev::spark::ClosedLoopConfig::FeedbackSensor>
        (sparkConfig.closedLoop.kAbsoluteEncoder);
    // sparkConfig.encoder.CountsPerRevolution(createInfo.feedbackConfig.countPerRev);
    break;
  }

  sparkConfig.closedLoop.SetFeedbackSensor(*encoder);

  // Limit Switch Configuaration
  switch (createInfo.feedbackConfig.forwardSwitch) {
  case LimitSwitchConfig::Disabled:
    sparkConfig.limitSwitch.ForwardLimitSwitchEnabled(false); 
    
    break;
  case LimitSwitchConfig::NormalyOpen:
    sparkConfig.limitSwitch.ForwardLimitSwitchType(rev::spark::LimitSwitchConfig::kNormallyOpen);
    sparkConfig.limitSwitch.ForwardLimitSwitchEnabled(true); 
    // sparkMax.GetForwardLimitSwitch(rev::spark::SparkLimitSwitchConfig::Type::kNormallyOpen)
    //     .EnableLimitSwitch(true);
    break;
  case LimitSwitchConfig::NormalyClosed:
    sparkConfig.limitSwitch.ForwardLimitSwitchType(rev::spark::LimitSwitchConfig::kNormallyClosed);
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
    sparkConfig.limitSwitch.ReverseLimitSwitchType(rev::spark::LimitSwitchConfig::kNormallyOpen);
    sparkConfig.limitSwitch.ReverseLimitSwitchEnabled(true); 
    // sparkMax.GetReverseLimitSwitch(rev::SparkLimitSwitch::Type::kNormallyOpen)
    //     .EnableLimitSwitch(true);
    break;
  case LimitSwitchConfig::NormalyClosed:
    sparkConfig.limitSwitch.ReverseLimitSwitchType(rev::spark::LimitSwitchConfig::kNormallyClosed);
    sparkConfig.limitSwitch.ReverseLimitSwitchEnabled(true); 
    // sparkMax.GetReverseLimitSwitch(rev::SparkLimitSwitch::Type::kNormallyClosed)
    //     .EnableLimitSwitch(true);
    break;
  }

  // Follower Congiguration
  for (auto &follower : createInfo.followers) {
    rev::spark::SparkMaxConfig followerConfig; 
    followers.emplace_back(
        std::make_unique<rev::spark::SparkMax>(follower.id, follower.motorType));
    followerConfig.Apply(sparkConfig).Follow(sparkMax);
    followers.back()->Configure(
        followerConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kPersistParameters);
    // followers.back()->Follow(sparkMax, follower.inverted);
  }

  sparkMax.Configure(sparkConfig,rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                    rev::spark::SparkBase::PersistMode::kPersistParameters);
}

void SparkMaxVelocityController::setVelocity(
    units::radians_per_second_t velocity) {
  targetVelocity = velocity;
  sparkMax.GetClosedLoopController().SetReference(
      units::revolutions_per_minute_t(targetVelocity).to<double>() * gearRatio,
      controlType);
}

units::radians_per_second_t
SparkMaxVelocityController::getTargetVelocity() const {
  return targetVelocity;
}

void SparkMaxVelocityController::setPower(double power) {
  targetVelocity = 0.0_rad_per_s;
  sparkMax.Set(power);
}

double SparkMaxVelocityController::getPower() const { return sparkMax.Get(); }

void SparkMaxVelocityController::disable() {
  targetVelocity = 0.0_rad_per_s;
  sparkMax.Disable();
}

void SparkMaxVelocityController::stop() {
  targetVelocity = 0.0_rad_per_s;
  sparkMax.StopMotor();
}

units::radians_per_second_t SparkMaxVelocityController::getVelocity() const {
  using EncoderType = SparkMaxVelocityControllerHelper::EncoderType;

  switch (encoderType) {
  case EncoderType::HallSensor:
  case EncoderType::Quadrature: {
    rev::spark::SparkRelativeEncoder *rel =
        static_cast<rev::spark::SparkRelativeEncoder *>(&sparkMax.GetEncoder());
    return units::revolutions_per_minute_t(rel->GetVelocity() / gearRatio);
  }
  case EncoderType::Alternate: {
    rev::spark::SparkMaxAlternateEncoder *alt =
        static_cast<rev::spark::SparkMaxAlternateEncoder *>(&sparkMax.GetAlternateEncoder());
    return units::revolutions_per_minute_t(alt->GetVelocity() / gearRatio);
  }
  case EncoderType::Absolute: {
    rev::AbsoluteEncoder *ab =
        static_cast<rev::spark::SparkAbsoluteEncoder *>(&sparkMax.GetAbsoluteEncoder());
    return units::revolutions_per_minute_t(ab->GetVelocity() / gearRatio);
  }
  }

  return 0_rpm;
}

units::radian_t SparkMaxVelocityController::getPosition() const {
  using EncoderType = SparkMaxVelocityControllerHelper::EncoderType;

  switch (encoderType) {
  case EncoderType::HallSensor:
  case EncoderType::Quadrature: {
    rev::spark::SparkRelativeEncoder *rel =
        static_cast<rev::spark::SparkRelativeEncoder *>(&sparkMax.GetEncoder());
    return units::turn_t(rel->GetPosition() / gearRatio);
  }
  case EncoderType::Alternate: {
    rev::spark::SparkMaxAlternateEncoder *alt =
        static_cast<rev::spark::SparkMaxAlternateEncoder *>(&sparkMax.GetAlternateEncoder());
    return units::turn_t(alt->GetPosition() / gearRatio);
  }
  case EncoderType::Absolute: {
    rev::spark::SparkAbsoluteEncoder *ab =
        static_cast<rev::spark::SparkAbsoluteEncoder *>(&sparkMax.GetAbsoluteEncoder());
    return units::turn_t(ab->GetPosition() / gearRatio);
  }
  }

  return 0_rad;
}

void SparkMaxVelocityController::setEncoderPosition(units::radian_t position) {
  using EncoderType = SparkMaxVelocityControllerHelper::EncoderType;

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
        static_cast<rev::spark::SparkMaxAlternateEncoder *>(&sparkMax.GetAlternateEncoder());
    rel->SetPosition(units::turn_t(position).to<double>() * gearRatio);
    break;
  }
  case EncoderType::Absolute: {
    rev::spark::SparkAbsoluteEncoder *ab =
        static_cast<rev::spark::SparkAbsoluteEncoder *>(&sparkMax.GetAbsoluteEncoder());
    //  ab->SetZeroOffset(units::turn_t(position).to<double>() * gearRatio);
  }
  }
}

units::radians_per_second_t SparkMaxVelocityController::getTolerance() const {
  return tolerance;
}

} // namespace rmb
