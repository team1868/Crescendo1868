package frc.robot.parsers.json.utils;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class FeedForwardConstantsJson {
  public double s = 0.0;
  public double v = 0.0;
  public double a = 0.0;

  private SimpleMotorFeedforward ffConstants = null;

  public SimpleMotorFeedforward getSimpleMotorFeedforward() {
    if (ffConstants == null) {
      ffConstants = new SimpleMotorFeedforward(s, v, a);
    }
    return ffConstants;
  }
}
