package frc.robot.utils;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;

    public double rollPositionDeg = 0.0;
    public double pitchPositionDeg = 0.0;
    public double yawPositionDeg = 0.0;

    public double rollVelocityDegPerSec = 0.0;
    public double pitchVelocityDegPerSec = 0.0;
    public double yawVelocityDegPerSec = 0.0;
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}
