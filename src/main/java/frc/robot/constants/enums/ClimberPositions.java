package frc.robot.constants.enums;

import tagalong.measurements.Height;

public enum ClimberPositions implements Height {
  EXTEND(0.35),
  STOW(0.0),
  CLIMB_EXTEND(0.311303), // TODO tune
  CLIMB_RETRACT(0.0), // TODO tune
  AMP2_INIT(0.070511),
  AMP2_INTERMEDIATE(0.2),
  AMP2_FINAL(0.28),
  TRAP_INTERMEDIATE(0.2), // TODO tune
  TRAP_FINAL(0.28), // TODO tune
  TRAP_POSITION(0.398); // TODO tune

  private final double _meters;

  ClimberPositions(double meters) {
    _meters = meters;
  }

  @Override
  public double getHeightM() {
    return _meters;
  }
}
