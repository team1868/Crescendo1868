package frc.robot.parsers.json.utils;

public class FlywheelConfJson extends TagalongBaseJson {
  public String name;
  public DeviceJson flywheelMotor;
  public DeviceJson flywheelFollowerMotor;

  public int[][] gearRatio;
  public FeedForwardConstantsJson feedforward;
  public TrapezoidalLimitsJson trapezoidalLimits;
  public DefaultTolerancesJson defaultTolerances;
  public MotorControlJson flywheelMotorControl;
  public MotorControlJson flywheelFollowerControl;

  private double calculatedGearRatio = -1;

  public double getGearRatio() {
    if (calculatedGearRatio <= 0) {
      calculatedGearRatio = calculateRatio(gearRatio);
    }
    return calculatedGearRatio;
  }
}
