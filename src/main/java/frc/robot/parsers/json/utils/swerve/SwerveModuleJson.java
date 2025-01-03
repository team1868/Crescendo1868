package frc.robot.parsers.json.utils.swerve;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.parsers.json.utils.AbsoluteEncoderConfigJson;
import frc.robot.parsers.json.utils.DeviceJson;
import frc.robot.parsers.json.utils.FusedMotorJson;
import frc.robot.parsers.json.utils.TagalongBaseJson;
import frc.robot.parsers.json.utils.Translation2dJson;
import frc.robot.parsers.json.utils.UnitJson;

public class SwerveModuleJson extends TagalongBaseJson {
  private EncoderType encoderType = null;
  /**
   * Module id number
   */
  public int id;
  /**
   * Drive motor.
   */
  public DeviceJson drive;
  /**
   * Steer motor.
   */
  public DeviceJson unfusedSteer;
  /**
   * Absolute encoder.
   */
  public DeviceJson unfusedEncoder;
  /**
   * Fused steer motor and cancoder.
   */
  public FusedMotorJson fusedSteerDevices = null;
  /**
   * Absolute encoder.
   */
  public Translation2dJson moduleLocation;

  /**
   * Encoder offset
   */
  public AbsoluteEncoderConfigJson encoderConfig;

  private FeedbackConfigs steerFeedbackConfigs = null;

  public Translation2d getModuleLocation() {
    // Do unit stuff here
    return moduleLocation.getLengthM();
  }

  public static enum EncoderType {
    NONE,
    ANALOG,
    CANCODER,
    FUSED_CANCODER;
  }

  public EncoderType getEncoderType() {
    if (encoderType == null) {
      if (fusedSteerDevices != null)
        encoderType = EncoderType.FUSED_CANCODER;
      else if (unfusedEncoder == null)
        encoderType = EncoderType.NONE;
      else if (unfusedEncoder.type.equalsIgnoreCase("cancoder"))
        encoderType = EncoderType.CANCODER;
      else if (unfusedEncoder.type.equalsIgnoreCase("analog"))
        encoderType = EncoderType.ANALOG;
      else
        encoderType = EncoderType.NONE;
    }
    return encoderType;
  }

  public TalonFX getSteerMotorTalonFX() {
    return getEncoderType() == EncoderType.FUSED_CANCODER ? fusedSteerDevices.motor.getTalonFX()
                                                          : unfusedSteer.getTalonFX();
  }

  public FeedbackConfigs getFeedbackConfigs(double motorToWheelRatio, double encoderToWheelRatio) {
    if (steerFeedbackConfigs == null) {
      steerFeedbackConfigs = new FeedbackConfigs();
      if (getEncoderType() == EncoderType.FUSED_CANCODER) {
        steerFeedbackConfigs =
            steerFeedbackConfigs
                .withFeedbackSensorSource(fusedSteerDevices.getFusedDeviceSourceValue())
                .withFeedbackRemoteSensorID(fusedSteerDevices.encoder.id)
                .withRotorToSensorRatio((motorToWheelRatio / encoderToWheelRatio))
                .withSensorToMechanismRatio(encoderToWheelRatio)
                .withFeedbackRotorOffset(0.0);
      }
    }
    return steerFeedbackConfigs;
  }
}
