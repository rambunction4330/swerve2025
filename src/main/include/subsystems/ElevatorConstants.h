

#include "rev/SparkMax.h"
#include "rev/config/SparkParameters.h"
#include "rmb/motorcontrol/Talon/TalonFXPositionController.h"
#include "rmb/motorcontrol/sparkmax/SparkMaxPositionController.h"
#include "rmb/motorcontrol/sparkmax/SparkMaxVelocityController.h"
#include "units/angle.h"
#include "units/base.h"
#include "units/length.h"
namespace constants {
namespace elevator {

const rmb::TalonFXPositionControllerHelper::PIDConfig elevatorPID{
    .p = 0.01, .i = 0.000015, .d = 0.0, .kG = 0.032};
const units::turn_t maxTurns = 257.5_tr;
const units::meter_t maxHeight = 68.5_cm;
const auto linearToAngularGearRatio = maxHeight / maxTurns;
const rmb::TalonFXPositionController::CreateInfo elevatorControllerCreateInfo1{
    .config{.id = 60, .inverted = true, .brake = true},
    .pidConfig{elevatorPID},
    .range{.minPosition = 0.0_tr,
           .maxPosition = maxTurns,
           .continuousWrap = false},
    .feedbackConfig{.sensorToMechanismRatio = (double)1 / 3 /*GearRatio*/},
    .openLoopConfig{},
    .currentLimits{30_A}};
const rmb::TalonFXPositionController::CreateInfo elevatorControllerCreateInfo2{
    .config{.id = 61, .inverted = false, .brake = false},
    .pidConfig{elevatorPID},
    .range{.minPosition = 0.0_tr,
           .maxPosition = maxTurns,
           .continuousWrap = false},
    .feedbackConfig{.sensorToMechanismRatio = (double)1 / 3 /*GearRatio*/},
    .openLoopConfig{},
    .currentLimits{30_A}};

} // namespace elevator

namespace wrist {
const double wrist_kG = -0.15;
const rmb::SparkMaxPositionController::CreateInfo
    wristPositionControllerCreateInfo{
        .motorConfig{.id = 50,
                     .motorType = rev::spark::SparkMax::MotorType::kBrushless,
                     .inverted = true},
        .pidConfig{.p = 0.04,
                   .i = 0.0,
                   .d = 0.00,
                   .ff = 0.015,
                   .tolerance = 2_tr,
                   .iZone = 0.0,
                   .iMaxAccumulator = 0.0,
                   .maxOutput = 1.0,
                   .minOutput = -1.0},

        .range{
            .minPosition = 0.0_tr, .maxPosition = 1_tr, .isContinuous = false},
        .profileConfig{},
        .feedbackConfig{
            .gearRatio = 25,

        },
        .followers{},

    };

const rmb::SparkMaxVelocityController::CreateInfo intakeVelocityController{
    .motorConfig =
        {
            .id = 51,
            .motorType = rev::spark::SparkMax::MotorType::kBrushless,
            .inverted = false,
        },
    .pidConfig{
        .p = 1.0,
        .i = 0.0,
        .d = 0.0,
        .ff = 0.0,
        .tolerance = 0.0_rad_per_s,
        .iZone = 0.0,
        .iMaxAccumulator = 0.0,
        .maxOutput = 1.0,
        .minOutput = -1.0,
    },
    .profileConfig{
        .useSmartMotion = false,
        .maxVelocity = 0.0_rpm,
        .minVelocity = 0.0_rpm,
        .maxAcceleration = 0.0_rad_per_s_sq,
    },
    .feedbackConfig{
        .gearRatio = 9,
        .encoderType =
            rmb::SparkMaxVelocityControllerHelper::EncoderType::HallSensor}};

} // namespace wrist

namespace climber {

const rmb::SparkMaxVelocityController::CreateInfo climberVelocityController1{
    .motorConfig =
        {
            .id = 56,
            .motorType = rev::spark::SparkMax::MotorType::kBrushless,
            .inverted = true,
        },
    .pidConfig{
        .p = 1.0,
        .i = 0.0,
        .d = 0.0,
        .ff = 0.0,
        .tolerance = 0.0_rad_per_s,
        .iZone = 0.0,
        .iMaxAccumulator = 0.0,
        .maxOutput = 1.0,
        .minOutput = -1.0,
    },
    .profileConfig{
        .useSmartMotion = false,
        .maxVelocity = 0.0_rpm,
        .minVelocity = 0.0_rpm,
        .maxAcceleration = 0.0_rad_per_s_sq,
    },
    .feedbackConfig{
        .gearRatio = 81,
        .encoderType =
            rmb::SparkMaxVelocityControllerHelper::EncoderType::HallSensor},
  };

const rmb::SparkMaxVelocityController::CreateInfo climberVelocityController2{
    .motorConfig =
        {
            .id = 57,
            .motorType = rev::spark::SparkMax::MotorType::kBrushless,
            .inverted = false,
        },
    .pidConfig{
        .p = 1.0,
        .i = 0.0,
        .d = 0.0,
        .ff = 0.0,
        .tolerance = 0.0_rad_per_s,
        .iZone = 0.0,
        .iMaxAccumulator = 0.0,
        .maxOutput = 1.0,
        .minOutput = -1.0,
    },
    .profileConfig{
        .useSmartMotion = false,
        .maxVelocity = 0.0_rpm,
        .minVelocity = 0.0_rpm,
        .maxAcceleration = 0.0_rad_per_s_sq,
    },
    .feedbackConfig{
        .gearRatio = 81,
        .encoderType =
            rmb::SparkMaxVelocityControllerHelper::EncoderType::HallSensor}};

} // namespace climber

} // namespace constants