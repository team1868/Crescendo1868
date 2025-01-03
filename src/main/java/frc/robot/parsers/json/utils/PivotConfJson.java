package frc.robot.parsers.json.utils;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class PivotConfJson extends TagalongBaseJson {
  public String name;
  public DeviceJson unfusedMotor = null;
  public DeviceJson unfusedEncoder = null;
  public FusedMotorJson fusedDevices = null;

  public ArbitraryFeedforwardJson feedforward;
  public TrapezoidalLimitsJson trapezoidalLimits;
  public PositionalLimitsJson positionalLimits;

  public DefaultTolerancesJson defaultTolerances;
  public MotorControlJson pivotMotorControl;
  public AbsoluteEncoderConfigJson encoderConfig;

  public int[][] motorToPivot;
  public int[][] encoderToPivot;

  public UnitJson ffOffset = UnitJson.defaultDegrees();
  public UnitJson profileOffset = UnitJson.defaultRotations();

  private double calculatedMotorToMechanismRatio = -1;
  private double calculatedEncoderToMechanismRatio = -1;
  private TalonFXConfiguration pivotMotorConfig = null;
  private FeedbackConfigs pivotFeedbackConfigs = null;
  private ClosedLoopGeneralConfigs closedLoopConfigs = null;

  public TalonFX getTalonFX() {
    return fusedDevices != null ? fusedDevices.motor.getTalonFX() : unfusedMotor.getTalonFX();
  }

  public CANcoder getCancoder() {
    return fusedDevices != null ? fusedDevices.encoder.getCancoder() : unfusedEncoder.getCancoder();
  }

  public double getMotorToMechanismRatio() {
    if (calculatedMotorToMechanismRatio <= 0) {
      calculatedEncoderToMechanismRatio = calculateRatio(motorToPivot);
    }
    return calculatedEncoderToMechanismRatio;
  }

  public double getEncoderToMechanismRatio() {
    if (calculatedMotorToMechanismRatio <= 0) {
      calculatedEncoderToMechanismRatio = calculateRatio(encoderToPivot);
    }
    return calculatedEncoderToMechanismRatio;
  }

  public CANcoderConfiguration getPivotCancoderConfiguration() {
    return encoderConfig.getCancoderConfig();
  }

  public TalonFXConfiguration getPrimaryMotorConfiguration() {
    if (pivotMotorConfig == null) {
      pivotMotorConfig =
          pivotMotorControl.getFullMotorConfiguration().withFeedback(getPivotFeedbackConfigs());
    }
    return pivotMotorConfig;
  }

  public FeedbackConfigs getPivotFeedbackConfigs() {
    if (pivotFeedbackConfigs == null) {
      pivotFeedbackConfigs = new FeedbackConfigs();
      if (fusedDevices != null) {
        pivotFeedbackConfigs =
            pivotFeedbackConfigs.withFeedbackSensorSource(fusedDevices.getFusedDeviceSourceValue())
                .withFeedbackRemoteSensorID(fusedDevices.encoder.id)
                .withRotorToSensorRatio(getMotorToMechanismRatio() / getEncoderToMechanismRatio())
                .withSensorToMechanismRatio(getEncoderToMechanismRatio())
                .withFeedbackRotorOffset(0.0);
      }
    }
    return pivotFeedbackConfigs;
  }
}
