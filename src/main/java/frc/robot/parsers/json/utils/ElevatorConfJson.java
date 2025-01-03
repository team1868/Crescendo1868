package frc.robot.parsers.json.utils;

public class ElevatorConfJson extends TagalongBaseJson {
  public String name;
  public DeviceJson motor;
  public int[][] gearRatio;
  public UnitJson drumDiameter;
  private double drumCircumference = 0;
  public ArbitraryFeedforwardJson feedforward;
  public TrapezoidalLimitsJson trapezoidalLimits;
  public PositionalLimitsJson positionalLimits;
  public DefaultTolerancesJson defaultTolerances;
  public MotorControlJson motorControl;
  private double calculatedGearRatio = -1;
  public DeviceJson followerMotor;
  public MotorControlJson followerMotorControl;

  public double getDrumCircumference() {
    if (drumCircumference <= 0) {
      drumCircumference = Math.PI * drumDiameter.getLengthM();
    }

    return drumCircumference;
  }

  public double getGearRatio() {
    if (calculatedGearRatio <= 0) {
      calculatedGearRatio = calculateRatio(gearRatio);
    }
    return calculatedGearRatio;
  }
}
