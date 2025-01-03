package frc.robot.utils;

public class TagalongMinorSystemBase {
  /* -------- MinorSystem specific disablements -------- */
  // Configured disablement for explicit on off
  public final boolean _configuredMinorSystemDisable;
  // Disablement state -- based off hardware disconnects
  protected boolean _isMinorSystemDisabled = true;

  // null parser is a configured disablement
  public TagalongMinorSystemBase(Object parser) {
    _configuredMinorSystemDisable = parser == null;
    // In the future we need a more discrete function configuring this
    _isMinorSystemDisabled = _configuredMinorSystemDisable;
  }

  public boolean motorResetConfig() {
    return !_isMinorSystemDisabled;
  }

  public boolean checkInitStatus() {
    return !_isMinorSystemDisabled;
  }

  /* -------- Math utilities -------- */
  public double clamp(double target, double min, double max) {
    return Math.max(min, Math.min(max, target));
  }

  public boolean inTolerance(double position, double lowerBound, double upperBound) {
    return (position >= lowerBound) && (position <= upperBound);
  }
}
