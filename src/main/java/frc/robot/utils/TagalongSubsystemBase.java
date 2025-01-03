package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TagalongSubsystemBase extends SubsystemBase {
  /* -------- Subsystem specific disablements -------- */
  // Configured disablement for explicit on off
  public final boolean _configuredDisable;
  // Disablement state -- based off hardware disconnects
  protected boolean _isSubsystemDisabled = true;

  // null parser is a configured disablement
  public TagalongSubsystemBase(Object parser) {
    _configuredDisable = parser == null;
    // In the future we need a more discrete function configuring this
    _isSubsystemDisabled = _configuredDisable;
  }

  public void setDisabled(boolean disable) {
    _isSubsystemDisabled = disable || _configuredDisable;
  }

  public boolean isSubsystemDisabled() {
    return _isSubsystemDisabled;
  }

  /* -------- IO and config functions -------- */
  protected void updateShuffleboard() {}
  protected void configShuffleboard() {}
  public void onEnable() {}
  public void onDisable() {}
  public void disabledPeriodic() {}
  public void simulationInit() {}
  public void simulationPeriodic() {}

  public void configMotor() {}

  public boolean checkInitStatus() {
    return !_isSubsystemDisabled;
  }

  public double clamp(double target, double min, double max) {
    return Math.max(min, Math.min(max, target));
  }
}
