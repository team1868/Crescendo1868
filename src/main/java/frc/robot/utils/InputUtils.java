package frc.robot.utils;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Constants.Control;

public class InputUtils {
  public static final double DENOMINATOR = 1.0 - Control.STICK_DEADBAND;
  /* For the scalings in both directions, I used the deadband constants
   which had comments on them saying they needed to be tuned */

  public static double scaleJoystickXMPS(double rawXAxis, double hypot, double maxSpeedMPS) {
    double cosine = hypot > 0.0000001 ? rawXAxis / hypot : 0.0;
    double scale = (hypot - Control.STICK_DEADBAND) * cosine / DENOMINATOR;
    return scale * maxSpeedMPS;
  }

  public static double scaleJoystickYMPS(double rawYAxis, double hypot, double maxSpeedMPS) {
    double sine = hypot > 0.0000001 ? rawYAxis / hypot : 0.0;
    double scale = (hypot - Control.STICK_DEADBAND) * sine / DENOMINATOR;
    return scale * maxSpeedMPS;
  }

  /**
   * Apply deadband. Ramps linearly from 0 to 1 from (deadband, 1) and 0 to -1
   * from (-deadband, -1).
   */
  public static double applyDeadband(final double value, final double deadband) {
    if (Math.abs(value) < deadband) {
      return 0.0;
    } else {
      final double signedOne = Math.signum(value);
      return ((value - signedOne) / (1.0 - deadband)) + signedOne;
    }
  }

  public static double ScaleJoystickThetaRadPS(double rawAxis, Rotation2d maxAngularSpeed) {
    return ScaleJoystickThetaRadPS(rawAxis, maxAngularSpeed.getRadians());
  }

  public static double ScaleJoystickThetaRadPS(double rawAxis, double maxAngularSpeedRadPS) {
    // double angular = Math.abs(rawAxis) < Control.STICK_DEADBAND ? 0 : rawAxis;
    double angular = Math.abs(rawAxis) < Control.STICK_DEADBAND
        ? 0.0
        : Math.signum(rawAxis) * Math.pow(rawAxis, 2);
    // double angular = Math.abs(rawAxis) < Control.STICK_DEADBAND ? 0 :
    // Math.pow(rawAxis, 3);
    return angular * maxAngularSpeedRadPS;
  }
}
