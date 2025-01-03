package frc.robot.parsers.json.utils;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;

public class ClosedLoopConfigsJson {
  public double dutyCycleClosedLoopRampPeriod = 0.0;
  public double torqueClosedLoopRampPeriod = 0.0;
  public double voltageClosedLoopRampPeriod = 0.0;
  public boolean continuousWrap = false;

  private ClosedLoopRampsConfigs closedLoopRampsConfigs = null;
  private ClosedLoopGeneralConfigs closedLoopGeneralConfigs = null;

  public ClosedLoopRampsConfigs getClosedLoopRampsConfigs() {
    if (closedLoopRampsConfigs == null) {
      closedLoopRampsConfigs = new ClosedLoopRampsConfigs()
                                   .withDutyCycleClosedLoopRampPeriod(dutyCycleClosedLoopRampPeriod)
                                   .withTorqueClosedLoopRampPeriod(torqueClosedLoopRampPeriod)
                                   .withVoltageClosedLoopRampPeriod(voltageClosedLoopRampPeriod);
    }
    return closedLoopRampsConfigs;
  }

  public ClosedLoopGeneralConfigs getClosedLoopGeneralConfigs() {
    if (closedLoopGeneralConfigs == null) {
      closedLoopGeneralConfigs = new ClosedLoopGeneralConfigs();
      closedLoopGeneralConfigs.ContinuousWrap = continuousWrap;
    }
    return closedLoopGeneralConfigs;
  }
}
