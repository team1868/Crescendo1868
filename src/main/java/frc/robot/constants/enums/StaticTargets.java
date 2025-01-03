package frc.robot.constants.enums;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.AlliancePose2d;

public enum StaticTargets {
  ENGAGE_CHAIN_1(new Pose2d(), new Pose2d()),
  ENGAGE_CHAIN_2(new Pose2d(), new Pose2d()),
  ENGAGE_CHAIN_3(new Pose2d(), new Pose2d()),
  STAGE_ALIGNED_1(new Pose2d(), new Pose2d()),
  STAGE_ALIGNED_2(new Pose2d(), new Pose2d()),
  STAGE_ALIGNED_3(new Pose2d(), new Pose2d()),
  AMP(new Pose2d(), new Pose2d()),
  SOURCE(
      new Pose2d(
          Units.inchesToMeters(615.445), Units.inchesToMeters(22.235), Rotation2d.fromDegrees(120.0)
      ),
      new Pose2d(
          Units.inchesToMeters(35.78), Units.inchesToMeters(22.235), Rotation2d.fromDegrees(60.0)
      )
  ),
  SOURCE_INTAKE(
      new Pose2d(
          Units.inchesToMeters(615.445 - 13.0),
          Units.inchesToMeters(22.235 - 13.0),
          Rotation2d.fromDegrees(120.0)
      ),
      new Pose2d(
          Units.inchesToMeters(35.78 + 13.0),
          Units.inchesToMeters(22.235 + 13.0),
          Rotation2d.fromDegrees(60.0)
      )
  );

  public final AlliancePose2d target;

  StaticTargets(Pose2d blue, Pose2d red) {
    this.target = new AlliancePose2d(blue, red);
  }

  StaticTargets(AlliancePose2d target) {
    this.target = target;
  }

  public Pose2d getPose2d(boolean isRedAlliance) {
    return target.get(isRedAlliance);
  }

  public Translation2d getTranslation2d(boolean isRedAlliance) {
    return target.get(isRedAlliance).getTranslation();
  }
}
