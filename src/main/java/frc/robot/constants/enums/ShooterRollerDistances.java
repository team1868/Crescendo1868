package frc.robot.constants.enums;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.TagalongAngle;

public enum ShooterRollerDistances implements TagalongAngle {
  MOVE_NOTE_BACK(0.0), // TODO tune all
  MOVE_NOTE_FORWARD(2.0),
  TRAP_PREP(-2.0),
  AMP_PREP(-10.0),
  AMP_SHOT_DISTANCE(-3.7),

  PASS_NOTE_TO_ARM(5.0);

  private final Rotation2d _value;

  ShooterRollerDistances(double rotations) {
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
