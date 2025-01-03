package frc.robot.constants.enums;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

public enum FieldVersions {
  NASA_FIELD(
      "/configs/vision/apriltag/visionTargetsNASA.json",
      FieldConstants.THEORETICAL_X_IN - 72.25,
      FieldConstants.THEORETICAL_Y_IN - 30.5
  ),
  THEORETICAL_FIELD(FieldConstants.THEORETICAL_X_IN, FieldConstants.THEORETICAL_Y_IN),
  COLORADO_FIELD(FieldConstants.THEORETICAL_X_IN, FieldConstants.THEORETICAL_Y_IN),
  SVR_FIELD(FieldConstants.THEORETICAL_X_IN, FieldConstants.THEORETICAL_Y_IN);

  public final String _visionFieldFile;
  public final double xM, yM;
  public final double halfXM, halfYM;
  // Assume lost distance is caused by shortening rather than negative offsets
  public final double deltaXM, deltaYM;

  FieldVersions(double fieldXIn, double fieldYIn) {
    this("/configs/vision/apriltag/2024-crescendo.json", fieldXIn, fieldYIn);
  }

  FieldVersions(String visionConfFile, double fieldXIn, double fieldYIn) {
    _visionFieldFile = Filesystem.getDeployDirectory() + visionConfFile;
    xM = Units.inchesToMeters(fieldXIn);
    yM = Units.inchesToMeters(fieldYIn);
    halfXM = xM / 2.0;
    halfYM = yM / 2.0;
    deltaXM = FieldConstants.THEORETICAL_X_M - xM;
    deltaYM = FieldConstants.THEORETICAL_Y_M - yM;
  }

  public static final class FieldConstants {
    public static final double THEORETICAL_X_IN = 651.25,
                               THEORETICAL_X_M = Units.inchesToMeters(THEORETICAL_X_IN);
    public static final double THEORETICAL_Y_IN = 315.5,
                               THEORETICAL_Y_M = Units.inchesToMeters(THEORETICAL_Y_IN);
    public static final double TAPE_WIDTH_IN = 2.0,
                               TAPE_WIDTH_M = Units.inchesToMeters(TAPE_WIDTH_IN);
  }
}
