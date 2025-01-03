package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;

public class AllianceTranslation2d {
  public final Translation2d blue, red;

  public AllianceTranslation2d(Translation2d blue, Translation2d red) {
    this.blue = blue;
    this.red = red;
  }

  public Translation2d get(boolean isRed) {
    return isRed ? red : blue;
  }
}
