package frc.robot.constants.enums;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Controlboard;
import tagalong.measurements.AlliancePose3d;
import tagalong.measurements.AllianceTranslation2d;

public enum ScoreTargets {
  AMP_CORNER(
      new Pose3d(
          Units.inchesToMeters(0.0),
          Units.inchesToMeters(FieldDims.THEORETICAL_Y_M), // TODO check
          Units.inchesToMeters(0.0),
          new Rotation3d(0.0, 0.0, 0.0)
      ),
      new Pose3d(
          FieldDims.THEORETICAL_X_M,
          FieldDims.THEORETICAL_Y_M, // TODO check
          Units.inchesToMeters(0.0),
          new Rotation3d(0.0, 0.0, 0.0)
      )
  ),
  AMP_LAUNCH_TARGET(
      new Pose3d(
          Units.inchesToMeters(0.0),
          FieldDims.THEORETICAL_Y_M - 0.5, // TODO check
          Units.inchesToMeters(0.0),
          new Rotation3d(0.0, 0.0, 0.0)
      ),
      new Pose3d(
          FieldDims.THEORETICAL_X_M,
          FieldDims.THEORETICAL_Y_M - 1.5, // TODO check
          Units.inchesToMeters(0.0),
          new Rotation3d(0.0, 0.0, 0.0)
      )
  ),
  AMP_LEFT_LAUNCH_TARGET(
      new Pose3d(
          Units.inchesToMeters(0.0),
          Units.inchesToMeters(218.42) + SpeakerConstants.SPEAKER_WIDTH_MINUS_TOLERANCE_DIV2
              + 1, // TODO check
          Units.inchesToMeters(78.0),
          new Rotation3d(0.0, 0.0, 180.0)
      ),
      new Pose3d(
          FieldDims.THEORETICAL_X_M,
          FieldDims.THEORETICAL_Y_M, // TODO check
          Units.inchesToMeters(0.0),
          new Rotation3d(0.0, 0.0, 0.0)
      )
  ),
  AMP_RIGHT_LAUNCH_TARGET(
      new Pose3d(
          Units.inchesToMeters(0.0),
          FieldDims.THEORETICAL_Y_M + 1, // TODO check
          Units.inchesToMeters(0.0),
          new Rotation3d(0.0, 0.0, 0.0)
      ),
      new Pose3d(
          Units.inchesToMeters(651.25),
          Units.inchesToMeters(218.42)
              + SpeakerConstants.SPEAKER_WIDTH_MINUS_TOLERANCE_DIV2, // TODO
                                                                     // check
          Units.inchesToMeters(78.0),
          new Rotation3d(0.0, 0.0, 0.0)
      )
  ),
  AMP_CORNER_RIGHT(
      new Pose3d(
          Units.inchesToMeters(0.0),
          Units.inchesToMeters(1.0), // TODO check
          Units.inchesToMeters(0.0),
          new Rotation3d(0.0, 0.0, 0.0)
      ),
      new Pose3d(
          FieldDims.THEORETICAL_X_M,
          FieldDims.THEORETICAL_Y_M - 1.0, // TODO check
          Units.inchesToMeters(0.0),
          new Rotation3d(0.0, 0.0, 0.0)
      )
  ),
  SPEAKER_MIDDLE(
      new Pose3d(
          Units.inchesToMeters(9.0),
          Units.inchesToMeters(218.42), // TODO check
          Units.inchesToMeters(78.0 + 82.875) / 2.0,
          new Rotation3d(0.0, 0.0, 180.0)
      ),
      new Pose3d(
          Units.inchesToMeters(651.25 - 9.0),
          Units.inchesToMeters(218.42), // TODO check
          Units.inchesToMeters(78.0 + 82.875) / 2.0,
          new Rotation3d(0.0, 0.0, 0.0)
      )
  ),
  // Translation2d(X: -0.42, Y: 0.11)
  // Translation2d(X: 16.73, Y: 5.44)
  // 16.31
  SPEAKER_LEFT_MIDDLE(
      new Pose3d(
          Units.inchesToMeters(9.0),
          Units.inchesToMeters(218.42)
              - SpeakerConstants.SPEAKER_WIDTH_MINUS_TOLERANCE_DIV2, // TODO
                                                                     // check
          Units.inchesToMeters(78.0 + 82.875) / 2.0,
          new Rotation3d(0.0, 0.0, 180.0)
      ),
      new Pose3d(
          Units.inchesToMeters(651.25 - 9.0),
          Units.inchesToMeters(218.42)
              + SpeakerConstants.SPEAKER_WIDTH_MINUS_TOLERANCE_DIV2, // TODO
                                                                     // check
          Units.inchesToMeters(78.0 + 82.875) / 2.0,
          new Rotation3d(0.0, 0.0, 0.0)
      )
  ),
  SPEAKER_RIGHT_MIDDLE(
      new Pose3d(
          Units.inchesToMeters(9.0),
          Units.inchesToMeters(218.42)
              + SpeakerConstants.SPEAKER_WIDTH_MINUS_TOLERANCE_DIV2, // TODO
                                                                     // check
          Units.inchesToMeters(78.0 + 82.875) / 2.0,
          new Rotation3d(0.0, 0.0, 180.0)
      ),
      new Pose3d(
          Units.inchesToMeters(651.25 - 9.0),
          Units.inchesToMeters(218.42)
              - SpeakerConstants.SPEAKER_WIDTH_MINUS_TOLERANCE_DIV2, // TODO
                                                                     // check
          Units.inchesToMeters(78.0 + 82.875) / 2.0,
          new Rotation3d(0.0, 0.0, 0.0)
      )
  ),
  SPEAKER_BOTTOM(
      new Pose3d(
          Units.inchesToMeters(0.0),
          Units.inchesToMeters(218.42), // TODO check
          Units.inchesToMeters(78.0),
          new Rotation3d(0.0, 0.0, 180.0)
      ),
      new Pose3d(
          Units.inchesToMeters(651.25),
          Units.inchesToMeters(218.42), // TODO check
          Units.inchesToMeters(78.0),
          new Rotation3d(0.0, 0.0, 0.0)
      )
  ),
  SPEAKER_LEFT_BOTTOM(
      new Pose3d(
          Units.inchesToMeters(0.0),
          Units.inchesToMeters(218.42)
              - SpeakerConstants.SPEAKER_WIDTH_MINUS_TOLERANCE_DIV2, // TODO
                                                                     // check
          Units.inchesToMeters(78.0),
          new Rotation3d(0.0, 0.0, 180.0)
      ),
      new Pose3d(
          Units.inchesToMeters(651.25),
          Units.inchesToMeters(218.42)
              + SpeakerConstants.SPEAKER_WIDTH_MINUS_TOLERANCE_DIV2, // TODO
                                                                     // check
          Units.inchesToMeters(78.0),
          new Rotation3d(0.0, 0.0, 0.0)
      )
  ),
  SPEAKER_RIGHT_BOTTOM(
      new Pose3d(
          Units.inchesToMeters(0.0),
          Units.inchesToMeters(218.42)
              + SpeakerConstants.SPEAKER_WIDTH_MINUS_TOLERANCE_DIV2, // TODO
                                                                     // check
          Units.inchesToMeters(78.0),
          new Rotation3d(0.0, 0.0, 180.0)
      ),
      new Pose3d(
          Units.inchesToMeters(651.25),
          Units.inchesToMeters(218.42)
              - SpeakerConstants.SPEAKER_WIDTH_MINUS_TOLERANCE_DIV2, // TODO
                                                                     // check
          Units.inchesToMeters(78.0),
          new Rotation3d(0.0, 0.0, 0.0)
      )
  ),
  SPEAKER_TOP(
      new Pose3d(
          Units.inchesToMeters(18.0),
          Units.inchesToMeters(218.42), // TODO check
          Units.inchesToMeters(82.875),
          new Rotation3d(0.0, 0.0, 180.0)
      ),
      new Pose3d(
          Units.inchesToMeters(651.25 - 18.0),
          Units.inchesToMeters(218.42),
          Units.inchesToMeters(78.0),
          new Rotation3d(0.0, 0.0, 0.0)
      )
  ),
  AMP(new Pose3d(
          Units.inchesToMeters(0.0), // TODO fix
          (Units.inchesToMeters(0.0) + Units.inchesToMeters(0.0)) / 2.0,
          (Units.inchesToMeters(0.0) + Units.inchesToMeters(0.0)) / 2.0,
          new Rotation3d(0.0, 0.0, 90.0)
      ),
      new Pose3d(
          Units.inchesToMeters(0.0), // TODO fix
          (Units.inchesToMeters(0.0) + Units.inchesToMeters(0.0)) / 2.0,
          (Units.inchesToMeters(0.0) + Units.inchesToMeters(0.0)) / 2.0,
          new Rotation3d(0.0, 0.0, 90.0)
      ));

  public final AlliancePose3d target; // target at middle
  public double widthM;

  public static class SpeakerConstants {
    public static final double WIDTH_M = Units.inchesToMeters(41.38);
    public static final double SPEAKER_WIDTH_MINUS_TOLERANCE_DIV2 =
        (SpeakerConstants.WIDTH_M / 2.0) - SpeakerConstants.WIDTH_M / 4.0;
    public static final double LENGTH_M = Units.inchesToMeters(4.77);
    public static final double HOOD_LENGTH_M = Units.inchesToMeters(18.11);
    public static final double FACE_M = Math.hypot(HOOD_LENGTH_M, LENGTH_M);
    public static final double ANGLE_OF_ELEVATION_RAD = Math.atan(LENGTH_M / HOOD_LENGTH_M);
    // TODO: update tolerances
    public static final double SPEAKER_EDGE_TOLERANCE_M = Units.inchesToMeters(3.0);
    public static final double PITCH_TOLERANCE_M = 0.0;
  }

  ScoreTargets(Pose3d blue, Pose3d red) {
    this.target = new AlliancePose3d(blue, red);
  }

  ScoreTargets(AlliancePose3d target) {
    this.target = target;
  }

  public Translation3d getTranslation3d(boolean isRedAlliance) {
    return target.get(isRedAlliance).getTranslation();
  }

  public Translation2d getTranslation2d(boolean isRedAlliance) {
    return getTranslation3d(isRedAlliance).toTranslation2d();
  }

  public Pose2d getPose2d(boolean isRedAlliance) {
    return target.get(isRedAlliance).toPose2d();
  }
}
