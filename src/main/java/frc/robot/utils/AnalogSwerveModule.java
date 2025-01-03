package frc.robot.utils;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.RobotAltModes;
import frc.robot.parsers.SwerveParser;

// TODO verify functionality
public class AnalogSwerveModule extends TagalongSwerveModuleBase {
  /* ---  Analog IO encoder --- */
  private AnalogInput _analogInput;

  public AnalogSwerveModule(
      int moduleNumber, SwerveParser swerveParser, double theoreticalMaxWheelSpeedMPS
  ) {
    super(moduleNumber, swerveParser, theoreticalMaxWheelSpeedMPS);
  }

  @Override
  public double getRotationAngleEncoder() {
    return (_analogInput.getAverageVoltage() / RobotController.getVoltage5V())
        + _angleOffsetRotation;
  }

  @Override
  public void disabledPeriodic() {
    if (Double.isInfinite(_lastAngleRot)) {
      resetToAbsolute();
    }
  }

  @Override
  public void updateShuffleboard() {
    if (RobotAltModes.isVerboseMode) {
      super.updateShuffleboard();
      _shuffleboardModuleEncoder.setDouble(getDegreeAngleEncoder());
    }
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (_steerMotor.hasResetOccurred()) {
      resetToAbsolute();
    }

    super.setDesiredState(desiredState, isOpenLoop);
  }

  /* ---- Only here to be overidden but still called ---- */
  @Override
  protected void initEncoder() {
    _analogInput = new AnalogInput(_conf.unfusedEncoder.id);
  }

  @Override
  protected void configEncoder() {
    _analogInput.setAverageBits(2);
  }

  @Override
  public void resetToAbsolute() {
    // TODO xor logic w encoder clockwisePositive
    double rotation = _control.steerControl.clockwisePositive
            ^ (_conf.encoderConfig.getSensorDirection() == SensorDirectionValue.Clockwise_Positive)
        ? -getRotationAngleEncoder()
        : getRotationAngleEncoder();
    _lastAngleRot = rotation;

    _absolutePosition = rotation * _type.getSteerRatio();
    _steerMotor.setPosition(_absolutePosition);
  }

  @Override
  protected double getEncoderOffsetRot() {
    return _conf.encoderConfig.magnetOffset.getDistRotation().getRotations();
  }
}
