package frc.robot.parsers.json.utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class CurrentLimitsJson {
  public boolean statorEnableLimit;
  public double statorPeakLimit;
  public boolean supplyEnableLimit;
  public double supplyPeakLimit;
  public double supplyContinuousLimit;
  public double peakDuration;

  public CurrentLimitsConfigs getTalonFXCurrentLimits() {
    return new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(statorEnableLimit)
        .withStatorCurrentLimit(statorPeakLimit)
        .withSupplyCurrentLimitEnable(supplyEnableLimit)
        .withSupplyCurrentLimit(supplyPeakLimit)
        .withSupplyCurrentThreshold(supplyContinuousLimit)
        .withSupplyTimeThreshold(peakDuration);
  }
}
