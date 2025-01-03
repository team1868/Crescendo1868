package frc.robot.parsers.json.utils;

public class RollerConfJson extends TagalongBaseJson {
  public String name;
  public DeviceJson rollerMotor;
  public String pivotFile;
  public int[][] gearRatio;
  public FeedForwardConstantsJson feedforward;
  public TrapezoidalLimitsJson trapezoidalLimits;
  public DefaultTolerancesJson defaultTolerances;
  public MotorControlJson rollerMotorControl;

  private double calculatedGearRatio = -1;

  public double getGearRatio() {
    if (calculatedGearRatio <= 0) {
      calculatedGearRatio = calculateRatio(gearRatio);
    }
    return calculatedGearRatio;
  }
}
