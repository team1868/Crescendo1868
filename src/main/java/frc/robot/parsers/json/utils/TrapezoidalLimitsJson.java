package frc.robot.parsers.json.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class TrapezoidalLimitsJson {
  public String unit;
  public double velocity;
  public double acceleration;

  private TrapezoidProfile.Constraints parsedLimitsM = null;
  private Rotation2d veloRotation2d = null;
  private Rotation2d accelRotation2d = null;

  public TrapezoidProfile.Constraints getLimitsM() {
    if (parsedLimitsM == null) {
      if (unit.equalsIgnoreCase("inch")) {
        parsedLimitsM = new TrapezoidProfile.Constraints(
            Units.inchesToMeters(velocity), Units.inchesToMeters(acceleration)
        );
      } else if (unit.equalsIgnoreCase("meter")) {
        parsedLimitsM = new TrapezoidProfile.Constraints(velocity, acceleration);
      } else {
        System.err.println("Incompatible trapezoidal types: expected length but got " + unit);
        System.exit(1);
      }
    }
    return parsedLimitsM;
  }

  public TrapezoidProfile.Constraints getLimitsRad() {
    if (veloRotation2d == null || accelRotation2d == null) {
      accelRotation2d = UnitJson.getAngleRotation2d(unit, acceleration);
      veloRotation2d = UnitJson.getAngleRotation2d(unit, velocity);
    }
    return new TrapezoidProfile.Constraints(
        veloRotation2d.getRadians(), accelRotation2d.getRadians()
    );
  }

  public TrapezoidProfile.Constraints getLimitsDeg() {
    if (veloRotation2d == null || accelRotation2d == null) {
      accelRotation2d = UnitJson.getAngleRotation2d(unit, acceleration);
      veloRotation2d = UnitJson.getAngleRotation2d(unit, velocity);
    }
    return new TrapezoidProfile.Constraints(
        veloRotation2d.getDegrees(), accelRotation2d.getDegrees()
    );
  }
  public TrapezoidProfile.Constraints getLimitsRot() {
    if (veloRotation2d == null || accelRotation2d == null) {
      accelRotation2d = UnitJson.getAngleRotation2d(unit, acceleration);
      veloRotation2d = UnitJson.getAngleRotation2d(unit, velocity);
    }
    return new TrapezoidProfile.Constraints(
        veloRotation2d.getRotations(), accelRotation2d.getRotations()
    );
  }
}
