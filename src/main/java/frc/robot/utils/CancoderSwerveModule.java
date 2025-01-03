package frc.robot.utils;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.RobotAltModes;
import frc.robot.parsers.SwerveParser;

public class CancoderSwerveModule extends TagalongSwerveModuleBase {
  /* ---  Cancoder (CAN based) --- */
  private CANcoder _steerCancoder;

  public CancoderSwerveModule(
      int moduleNumber, SwerveParser swerveParser, double theoreticalMaxWheelSpeedMPS
  ) {
    super(moduleNumber, swerveParser, theoreticalMaxWheelSpeedMPS);
  }

  @Override
  protected void initEncoder() {
    _steerCancoder = _conf.unfusedEncoder.getCancoder();
  }

  @Override
  protected void configEncoder() {
    _steerCancoder.getConfigurator().apply(_swerveCancoderConfig);
  }

  @Override
  public double getRotationAngleEncoder() {
    return _steerCancoder.getPosition().getValueAsDouble();
  }

  @Override
  public boolean motorResetConfig() {
    if (_steerCancoder.hasResetOccurred()) {
      configEncoder();
      return super.motorResetConfig() || true;
    }
    return super.motorResetConfig();
  }

  @Override
  public void updateShuffleboard() {
    if (RobotAltModes.isVerboseMode) {
      super.updateShuffleboard();
      _shuffleboardModuleEncoder.setDouble(_steerCancoder.getAbsolutePosition().getValueAsDouble());
    }
  }

  @Override
  public void resetToAbsolute() {
    // TODO xor logic w encoder clockwisePositive
    double rotation = _control.steerControl.clockwisePositive
            ^ (_swerveCancoderConfig.MagnetSensor.SensorDirection
               == SensorDirectionValue.Clockwise_Positive)
        ? -getRotationAngleEncoder()
        : getRotationAngleEncoder();
    _lastAngleRot = rotation;

    _absolutePosition = rotation * _type.getSteerRatio();
    _steerMotor.setPosition(_absolutePosition);
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (_steerMotor.hasResetOccurred()) {
      resetToAbsolute();
    }

    super.setDesiredState(desiredState, isOpenLoop);
  }

  @Override
  protected double getEncoderOffsetRot() {
    return _conf.encoderConfig.magnetOffset.getDistRotation().getRotations();
  }
}
