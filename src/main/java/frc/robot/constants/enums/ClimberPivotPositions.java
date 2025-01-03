package frc.robot.constants.enums;

import edu.wpi.first.math.geometry.Rotation2d;
import tagalong.measurements.Angle;

public enum ClimberPivotPositions implements Angle {
  CLIMB_PREP(0.0), // TODO tune
  CLIMB(0.0); // TODO tune

  private final Rotation2d _value;

  ClimberPivotPositions(double degree) {
    _value = Rotation2d.fromDegrees(degree);
  }

  ClimberPivotPositions(Rotation2d value) {
    _value = value;
  }

  @Override
  public double getDegrees() {
    return _value.getDegrees();
  }

  @Override
  public double getRadians() {
    return _value.getRadians();
  }

  @Override
  public double getRotations() {
    return _value.getRotations();
  }
}
