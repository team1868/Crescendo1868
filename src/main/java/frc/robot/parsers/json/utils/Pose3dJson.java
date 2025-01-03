package frc.robot.parsers.json.utils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class Pose3dJson {
  private Pose3d pose;
  private Transform3d transform;
  public Translation3dJson translation = new Translation3dJson();
  public Rotation3dJson rotation = new Rotation3dJson();

  public Pose3d getPose3d() {
    if (pose == null) {
      pose = new Pose3d(translation.getLengthM(), rotation.getDistRotation());
    }
    return pose;
  }

  public Transform3d getTransform3d() {
    if (transform == null) {
      transform = getPose3d().minus(new Pose3d());
    }
    return transform;
  }
}
