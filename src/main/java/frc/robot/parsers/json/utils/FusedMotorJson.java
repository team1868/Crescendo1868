package frc.robot.parsers.json.utils;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

public class FusedMotorJson {
  public DeviceJson motor;
  public DeviceJson encoder;

  public FeedbackSensorSourceValue getFusedDeviceSourceValue() {
    if (encoder.type.equalsIgnoreCase("cancoder")) {
      return FeedbackSensorSourceValue.FusedCANcoder;

    } else {
      System.err.println(
          "ERROR: Invalid fused device! Expected cancoder but got a " + encoder.type
      );
      System.exit(1);
      return null;
    }
  }
}
