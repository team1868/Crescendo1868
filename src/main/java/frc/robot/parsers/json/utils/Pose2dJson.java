package frc.robot.parsers.json.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Pose2dJson {
  private Pose2d pose;
  private Transform2d transform;
  public Translation2dJson translation = new Translation2dJson();
  public Rotation2dJson rotation = new Rotation2dJson();

  public Pose2d getPose2d() {
    if (pose == null) {
      pose = new Pose2d(translation.getLengthM(), rotation.getDistRotation());
    }
    return pose;
  }

  public Transform2d getTransform2d() {
    if (transform == null) {
      transform = getPose2d().minus(new Pose2d());
    }
    return transform;
  }
}
