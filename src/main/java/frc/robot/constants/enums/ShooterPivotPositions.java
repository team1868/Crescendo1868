package frc.robot.constants.enums;

import edu.wpi.first.math.geometry.Rotation2d;
import tagalong.measurements.Angle;

public enum ShooterPivotPositions implements Angle {
  STOW(-50.0),
  PASS_OFF(-45.0),
  ZERO(0.0),
  SKIP_SHOT(-3.0),
  EJECT_NOTE(-5.0),

  // auto
  SPEAKER_SUBWOOFER_SHOT(-49.0),
  AUTONOMOUS_AMP_START_SPEAKER_SHOT(-49.0), // tune
  AUTONOMOUS_MID_START_SPEAKER_SHOT(-49.0),
  AUTONOMOUS_SOURCE_START_SPEAKER_SHOT(-41.0),

  N1_SHOT(-25),
  N2_SHOT(-19),
  N3_SHOT(-25),
  N4_SHOT(-25.5),
  N5_SHOT(-22),
  N6_SHOT(-22),
  N7_SHOT(-22),
  N8_SHOT(-22),

  AMP_FAR_SHOT(-17.2), // TODO
  SOURCE_FAR_SHOT(-17.2), // TODO: tuned from CO

  // ^^ above this is tuned, below is not 2/28
  SPEAKER(-17.0),
  SPEAKER_OFFSET(3.0), // TODO tune

  AMP_PREP(21.0), // TODO tune
  UNSAFE_AMP(11.0),
  AMP(29.0), // TODO tune
  AMP_UNTIL(15.0), // TODO tune

  AMP2_INIT(0.0),

  GROUND_TRAP(2.0), // TODO tune
  CLIMB_PREP(30.0), // TODO tune
  CLIMB_TRAP(30.0), // TODO tune

  SOURCE_INTAKE(-50.0),
  SOURCE_FEEDER_INTAKE(24.96096); // 30.0

  public final Rotation2d _value;

  ShooterPivotPositions(double degree) {
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
