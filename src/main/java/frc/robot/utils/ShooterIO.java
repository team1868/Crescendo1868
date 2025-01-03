package frc.robot.utils;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean connected = false;
    public boolean isPieceInIntake = false;

    public double pivotPositionRad = 0.0;
    public double pivotVelocityRadPerSec = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotCurrentAmps = 0.0;

    public double flywheelVelocityRadPerSec = 0.0;
    public double flywheelAppliedVolts = 0.0;
    public double flywheelCurrentAmps = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}
}
