package frc.robot.constants.enums;

import edu.wpi.first.math.geometry.Rotation2d;
import tagalong.measurements.Angle;

public enum IntakePivotPositions implements Angle {
  ZERO(0.0),
  STOW(65.0),
  PASS_OFF(17.0507808),
  PARTIAL_STOW(63.0),
  // ^^ above this is tuned, below is not 2/28
  RED_GROUND_INTAKE(-4.50),
  BLUE_GROUND_INTAKE(-4.75),
  CLEAR_OF_SHOOTER(21.0),
  SCORE_PREP(-10.0), // TODO tune
  AMP_PREP(60.0), // TODO tune
  AMP(60.0), // TODO tune
  CLIMB_PREP(-10.0), // TODO tune
  CLIMB(1.0), // TODO tune
  CLIMB_TRAP(1.0), // TODO tune
  EJECT_NOTE(5.0), // TODO tune
  AUTO_AIM_CLEARANCE(10.0),
  SOURCE_INTAKE(60.0),
  SOURCE_FEEDER_INTAKE(75.0),
  TEST(70.0); // TODO tune

  private final Rotation2d _value;

  IntakePivotPositions(double degree) {
    _value = Rotation2d.fromDegrees(degree);
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
