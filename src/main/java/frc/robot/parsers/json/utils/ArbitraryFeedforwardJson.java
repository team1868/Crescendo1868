package frc.robot.parsers.json.utils;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;

public class ArbitraryFeedforwardJson {
  public double s;
  public double g;
  public double v;
  public double a;

  public ArmFeedforward getArmFeedforward() {
    return new ArmFeedforward(s, g, v, a);
  }

  public ElevatorFeedforward getElevatorFeedforward() {
    return new ElevatorFeedforward(s, g, v, a);
  }
}
