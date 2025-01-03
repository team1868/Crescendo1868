package frc.robot.utils;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.constants.Constants;

public class PIDConstants {
  public final double p, i, d;
  public PIDConstants(double p, double i, double d) {
    this.p = p;
    this.i = i;
    this.d = d;
  }

  public static PIDConstants getUnprofiledController(double p, double i, double d) {
    return new PIDConstants(p, i, d);
  }

  public ProfiledPIDController getProfiledController(TrapezoidProfile.Constraints constraints) {
    return new ProfiledPIDController(
        p,
        i,
        d,
        new TrapezoidProfile.Constraints(constraints.maxVelocity, constraints.maxAcceleration),
        Constants.LOOP_PERIOD_S
    );
  }
}
