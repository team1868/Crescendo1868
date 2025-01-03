package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.enums.FieldVersions;

public class Constants {
  public static final RobotVersions CRobot = RobotVersions.COMP_BOT;
  public static final FieldVersions CField = FieldVersions.THEORETICAL_FIELD;

  // These are high level robot configurations that fundamentally change robot
  // behavior and control schemes, this also allows for manual overrides. I don't
  // like this,
  // would love to see alternatives... These should really become part of the top
  // level
  // RobotVersions

  // TODO move
  /* ========== ROBOT OFFSET DIMENSIONS ========== */
  public static final double LOOP_PERIOD_MS = 20.0;
  public static final double LOOP_PERIOD_S = Units.millisecondsToSeconds(LOOP_PERIOD_MS);

  /* ========== ROBOT SPEEDS ========== */
  public static final double NOTE_TRAVEL_INSIDE_DELTA_S = 1.0; // TODO: calculate

  /* ========== NATURAL CONSTANTS ========== */
  public static final double GRAVITY_MPS2 = -9.81;

  public static final class CTREConstants {
    /* ============= Falcon Constants2 ============= */
    // ticks per motor rotation
    // TODO: measure this free speed on blocks
    // 6380 +/- 10%
    // TODO FOC?
    public static final double MAX_KRAKEN_X60_FOC_FOC_RPM = 5800.0;
  }

  public static final class Sensors {
    public static final boolean INVERT_GYRO = false;

    public static final Rotation2d GYRO_ZERO_BLUE = Rotation2d.fromDegrees(0);
    public static final Rotation2d GYRO_ZERO_RED = Rotation2d.fromDegrees(180);

    public static final double POV_ZERO_BLUE_DEG = 0;
    public static final double POV_ZERO_RED_DEG = POV_ZERO_BLUE_DEG + 180;
  }

  /* For Drivetrain Auto */
  public static final double TRANSLATION_MAX_TRIM_SPEED_MPS = 1;
  public static final Rotation2d ANGLE_MAX_TRIM_SPEED_DPS = Rotation2d.fromDegrees(90.0);

  public static final class Control {
    public static final double STICK_DEADBAND = 0.04; // TODO: tune
    public static final double STICK_NET_DEADBAND = 0.06; // TODO: tune
    public static final boolean IS_OPEN_LOOP = false; // swerve
  }
}
