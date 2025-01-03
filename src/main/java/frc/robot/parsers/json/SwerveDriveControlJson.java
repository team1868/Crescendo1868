package frc.robot.parsers.json;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.parsers.json.utils.*;
import frc.robot.parsers.json.utils.swerve.SwerveTrapezoidalLimitsJson;

public class SwerveDriveControlJson {
  public SwerveTrapezoidalLimitsJson defaultLimits;
  public SwerveTrapezoidalLimitsJson defaultTrapezoidalLimits;

  public UnitJson translationalSlewingRate;
  public UnitJson angularSlewingRate;

  public UnitJson translationalTolerance;
  public UnitJson rotationalTolerance;

  public PIDSGVAConstantsJson translationalControl;
  public PIDSGVAConstantsJson rotationalControl;
  public PIDSGVAConstantsJson autoTranslationalControl;
  public PIDSGVAConstantsJson autoRotationalControl;

  public SlewRateLimiter getTranslationalSlewRateLimiter() {
    return new SlewRateLimiter(translationalSlewingRate.getAccelM());
  }

  public SlewRateLimiter getAngularSlewRateLimiter() {
    return new SlewRateLimiter(angularSlewingRate.getAccelRotation().getRotations());
  }
}
