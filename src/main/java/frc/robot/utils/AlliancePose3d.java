package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;

public class AlliancePose3d {
  public final Pose3d blue, red;

  public AlliancePose3d(Pose3d blue, Pose3d red) {
    this.blue = blue;
    this.red = red;
  }

  public Pose3d get(boolean isRed) {
    return isRed ? red : blue;
  }

  public Translation2d getTranslation2d(boolean isRed) {
    return get(isRed).getTranslation().toTranslation2d();
  }
}
