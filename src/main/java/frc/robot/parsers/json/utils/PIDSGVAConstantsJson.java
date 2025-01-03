package frc.robot.parsers.json.utils;

import frc.robot.utils.PIDSGVAConstants;

public class PIDSGVAConstantsJson {
  public double p = 0;
  public double i = 0;
  public double d = 0;
  public double s = 0;
  public double g = 0;
  public double v = 0;
  public double a = 0;

  public PIDSGVAConstants getPIDSGVAConstants() {
    return new PIDSGVAConstants(p, i, d, s, g, v, a);
  }
}
