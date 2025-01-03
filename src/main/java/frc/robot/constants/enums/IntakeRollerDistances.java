package frc.robot.constants.enums;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.TagalongAngle;

public enum IntakeRollerDistances implements TagalongAngle {
  EXAMPLE(0.0);

  private final Rotation2d _value;

  IntakeRollerDistances(double rotations) {
    _value = Rotation2d.fromRotations(rotations);
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
