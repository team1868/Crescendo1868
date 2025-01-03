package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Constants.Control;

public class InputUtils {
  public static final int XY_SCALING_POWER = 1;
  public static final boolean XY_SCALING = XY_SCALING_POWER > 1;
  public static final int THETA_SCALING_POWER = 1;
  public static final boolean THETA_SCALING = THETA_SCALING_POWER > 1;

  public static double scaleJoystickXMPS(double rawXAxis, double rawYAxis, double maxSpeedMPS) {
    double xyNet = Math.hypot(rawXAxis, rawYAxis);
    if (xyNet == 0.0) {
      return 0.0;
    }
    final double deadbandedR = applyDeadband(xyNet, Control.STICK_NET_DEADBAND);
    final double scaledCoefficient =
        (XY_SCALING ? Math.pow(deadbandedR, XY_SCALING_POWER) : deadbandedR) / xyNet;
    return rawXAxis * scaledCoefficient * maxSpeedMPS;
  }

  public static double scaleJoystickYMPS(double rawXAxis, double rawYAxis, double maxSpeedMPS) {
    double xyNet = Math.hypot(rawXAxis, rawYAxis);
    if (xyNet == 0.0) {
      return 0.0;
    }
    final double deadbandedR = applyDeadband(xyNet, Control.STICK_NET_DEADBAND);
    final double scaledCoefficient =
        (XY_SCALING ? Math.pow(deadbandedR, XY_SCALING_POWER) : deadbandedR) / xyNet;
    return rawYAxis * scaledCoefficient * maxSpeedMPS;
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
    double angular = Math.abs(rawAxis) < Control.STICK_DEADBAND
        ? 0.0
        : Math.signum(rawAxis) * (THETA_SCALING ? Math.pow(rawAxis, THETA_SCALING_POWER) : rawAxis);
    return angular * maxAngularSpeedRadPS;
  }
}
