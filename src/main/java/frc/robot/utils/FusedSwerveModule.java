package frc.robot.utils;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.RobotAltModes;
import frc.robot.parsers.SwerveParser;

public class FusedSwerveModule extends TagalongSwerveModuleBase {
  /* ---  Cancoder (CAN based) --- */
  private CANcoder _steerCancoder;

  public FusedSwerveModule(
      int moduleNumber, SwerveParser swerveParser, double theoreticalMaxWheelSpeedMPS
  ) {
    super(moduleNumber, swerveParser, theoreticalMaxWheelSpeedMPS);
    _lastAngleRot = _steerMotor.getPosition().getValueAsDouble();
  }

  @Override
  protected void initEncoder() {
    _steerCancoder = _conf.fusedSteerDevices.encoder.getCancoder();
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

  /* ---- Removed conversions involving steer ratio below ----*/
  @Override
  protected void setDesiredSteer() {
    _angleRot =
        Math.abs(_desiredState.speedMetersPerSecond) <= (_theoreticalMaxWheelSpeedMPS * 0.01)
        ? _lastAngleRot
        : _desiredState.angle.getRotations();

    // TODO: Reuse the position voltage object rather than creating a new one each time
    _steerMotor.setControl(_requestedSteerPositionVoltage.withPosition(_angleRot));

    _lastAngleRot = _angleRot;
  }

  @Override
  public double getDesiredSteer() {
    return _angleRot;
  }

  @Override
  public SwerveModuleState getState() {
    _curState.speedMetersPerSecond = RotorRPSToMPS(_driveMotor.getVelocity().getValueAsDouble());
    _curState.angle = Rotation2d.fromRotations(_steerMotor.getPosition().getValueAsDouble());

    return _curState;
  }

  @Override
  protected double getEncoderOffsetRot() {
    return _conf.encoderConfig.magnetOffset.getDistRotation().getRotations();
  }

  @Override
  public SwerveModulePosition getPosition() {
    _curPosition.distanceMeters = _driveMotor.getRotorPosition().getValueAsDouble()
        * _type.getWheelCircumferenceM() / _type.getDriveRatio();
    _curPosition.angle = Rotation2d.fromRotations(_steerMotor.getPosition().getValueAsDouble());

    return _curPosition;
  }

  @Override
  public SwerveModulePosition getPosition(boolean refresh) {
    if (refresh) {
      /* Refresh all signals */
      _drivePosition.refresh();
      _driveVelocity.refresh();
      _steerPosition.refresh();
      _steerVelocity.refresh();
    }

    /* Now latency-compensate our signals */
    double drive_rot = BaseStatusSignal.getLatencyCompensatedValue(_drivePosition, _driveVelocity);
    double steer_rot = BaseStatusSignal.getLatencyCompensatedValue(_steerPosition, _steerVelocity);

    /*
     * Back out the drive rotations based on angle rotations due to coupling between
     * azimuth and steer
     */
    drive_rot -= steer_rot * RobotAltModes.steerDriveCouplingRatio;

    /* And push them into a SwerveModulePosition object to return */
    _curPosition.distanceMeters =
        drive_rot * _type.getWheelCircumferenceM() / _type.getDriveRatio();
    // _curPosition.distanceMeters = drive_rot / _driveRotationsPerMeter;

    /* Angle is already in terms of steer rotations */
    _curPosition.angle = Rotation2d.fromRotations(steer_rot);

    return _curPosition;
  }
}
