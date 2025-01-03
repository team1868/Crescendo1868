package frc.robot.utils;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean connected = false;
    public boolean isPieceInIntake = false;

    public double pivotPositionRad = 0.0;
    public double pivotVelocityRadPerSec = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotCurrentAmps = 0.0;

    public double rollerVelocityRadPerSec = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerCurrentAmps = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}
}
