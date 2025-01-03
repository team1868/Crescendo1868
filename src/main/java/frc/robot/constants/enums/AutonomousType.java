package frc.robot.constants.enums;

public enum AutonomousType {
  BASIC_COMMAND(BuilderType.NOT_BUILDABLE),
  BASIC_CHOREO(BuilderType.CHOREO),
  SPLICED_CHOREO(BuilderType.CHOREO),
  BASIC_TAGALONG(BuilderType.TAGALONG),
  SPLICE_TAGALONG(BuilderType.TAGALONG),
  SPLICED_AND_MAPPED_TAGALONG(BuilderType.MAPPED_TAGALONG),
  SPLICED_AND_MAPPED_TAGALONG_WITH_FORCED_SHOT(BuilderType.MAPPED_TAGALONG);

  public final BuilderType _type;

  AutonomousType(BuilderType type) {
    _type = type;
  }

  public static enum BuilderType {
    TAGALONG,
    MAPPED_TAGALONG,
    CHOREO,
    NOT_BUILDABLE;
  }
}
