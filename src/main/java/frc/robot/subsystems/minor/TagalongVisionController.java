package frc.robot.subsystems.minor;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.Constants;
import frc.robot.constants.enums.PipelineIndex;
import frc.robot.parsers.VisionParser;
import frc.robot.parsers.json.utils.ValidTagsJson;
import frc.robot.utils.GeometricUtils;
import frc.robot.utils.LoopTimer;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class TagalongVisionController {
  public static final class VISION_CONSTANTS {
    public static final double MAX_TAG_RANGE_M = Units.feetToMeters(25.0);
    public static final double AUTO_VISION_SCALER = 10.0;
  }

  private String _cameraName;
  private PhotonCamera _camera;
  private Transform3d _robotToCamera;
  private PhotonPoseEstimator _visionPoseEstimator;
  private static int _columnCounter = 0;

  private final AprilTagFieldLayout _aprilTagFieldLayout;
  private GenericEntry _photonDistanceEntry;
  private GenericEntry _photonYawEntry;
  private GenericEntry _photonPitchEntry;

  private int _closestId = 0;
  public int _lastTargetCount = 0;
  private Pose3d _latestPose = new Pose3d();
  private static double _lastTimestamp = 0.0;
  private Matrix<N3, N1> _stdDevs;
  private Matrix<N3, N1> _maxDevs =
      VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
  private ValidTagsJson _validTagMap;
  private ArrayList<Integer> _tagsSeen;

  public TagalongVisionController(
      String cameraName,
      Transform3d robotToCamera,
      AprilTagFieldLayout aprilTagFieldLayout,
      ValidTagsJson validTagMap
  ) {
    _cameraName = cameraName;
    _robotToCamera = robotToCamera;
    _aprilTagFieldLayout = aprilTagFieldLayout;

    _camera = new PhotonCamera(_cameraName);
    _visionPoseEstimator = new PhotonPoseEstimator(
        _aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, _camera, _robotToCamera
    );
    _validTagMap = validTagMap;
    configShuffleboard();
  }

  private EstimatedRobotPose updatePoseEstimator() {
    var result = _camera.getLatestResult();
    List<PhotonTrackedTarget> targets = result.getTargets();
    if (targets.isEmpty()) {
      return null;
    }

    // For each target you see
    _tagsSeen = new ArrayList<Integer>();
    double area = targets.get(0).getArea();

    for (final PhotonTrackedTarget target : targets) {
      int id = target.getFiducialId();
      if (target.getArea() > area) {
        area = target.getArea();
        _closestId = id;
      }

      if (id > 16 || id < 1) {
        return null;
      }

      // For each previous tag seen (_tagsSeen) check if the current target is valid
      for (Integer tag : _tagsSeen) {
        if (_validTagMap.tags[tag - 1].isInvalid(id)) {
          // if it's invalid return null
          return null;
        }
      }
      _tagsSeen.add(id);
    }

    return _visionPoseEstimator.update(result).orElse(null);
  }

  public void updateVisionPose3d(Pose3d pose) {
    EstimatedRobotPose result = updatePoseEstimator();

    LoopTimer.markEvent("VISION 0");

    if (result == null) {
      _latestPose = pose;
      _stdDevs = _maxDevs;
      _lastTargetCount = -2; // -2 represents no images
      return;
    }

    LoopTimer.markEvent("VISION 0.5");

    LoopTimer.markEvent("VISION 1");

    var targetsUsed = result.targetsUsed;

    if (targetsUsed.size() <= 1) {
      _latestPose = pose;
      _stdDevs = _maxDevs;
      _lastTargetCount = -3;
      return;
    }

    PhotonTrackedTarget closestTarget = null;
    double smallestDistanceM = VisionParser.SINGLE_TAG_MAX_DIST_M;

    for (int i = 0; i < result.targetsUsed.size(); i++) {
      var target = targetsUsed.get(i);
      double distance = target.getBestCameraToTarget().getTranslation().getNorm();
      if (distance < smallestDistanceM && target.getPoseAmbiguity() < 0.2) {
        smallestDistanceM = distance;
        closestTarget = target;
      }
    }

    if (closestTarget == null || closestTarget.getPoseAmbiguity() >= 0.2) {
      _latestPose = pose;
      _stdDevs = _maxDevs;
      _lastTargetCount = -3; // -3 no target case (shouldn't happen) or ambiguous pose
      return;
    }

    LoopTimer.markEvent("VISION 2");
    LoopTimer.markEvent("VISION 3");

    _latestPose = new Pose3d(
        result.estimatedPose.getTranslation(),
        result.estimatedPose.getRotation().minus(new Rotation3d(
            0,
            0,
            Math.tan(
                closestTarget.getBestCameraToTarget().getY()
                / closestTarget.getBestCameraToTarget().getX()
            )
        ))
    );
    _lastTimestamp = result.timestampSeconds;
    _lastTargetCount = targetsUsed.size();
    updateVisionPoseStdDevs(closestTarget, smallestDistanceM, targetsUsed);
  }

  public static final double AMBIGUITY_SCALE = 160.0;
  public static final double AMBIGUITY_CONSTANT = 0.2;

  public static final double X_DISTANCE_SCALE = 0.0153; // 0.0000447 * 3;
  public static final double X_DISTANCE_POW = 0.257; // 1.81;

  public static final double Y_DISTANCE_SCALE = 0.0372; // 0.000271 * 3;
  public static final double Y_DISTANCE_POW = 0.371; // 1.99;

  public static final double THETA_SCALE = 95.2;
  public static final double THETA_CONSTANT = 56.4;

  public void updateVisionPoseStdDevs(
      PhotonTrackedTarget closestTarget,
      double smallestDistanceM,
      List<PhotonTrackedTarget> targetsUsed
  ) {
    if (smallestDistanceM > VISION_CONSTANTS.MAX_TAG_RANGE_M
        || (_visionPoseEstimator.getPrimaryStrategy() != PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
            && smallestDistanceM > VisionParser.SINGLE_TAG_MAX_DIST_M)) {
      _stdDevs = _maxDevs;
      // setting standard deviation to max if it is too far away
      return;
    }
    if (DriverStation.isAutonomousEnabled()) {
      if (targetsUsed.size() <= 1) {
        _stdDevs = _maxDevs;
        // setting standard deviation to max if it is too far away
        return;
      }
      // TODO: tune scales
      double xDev = X_DISTANCE_SCALE * Math.exp(X_DISTANCE_POW * smallestDistanceM)
          * VISION_CONSTANTS.AUTO_VISION_SCALER;
      double yDev = Y_DISTANCE_SCALE * Math.exp(Y_DISTANCE_POW * smallestDistanceM)
          * VISION_CONSTANTS.AUTO_VISION_SCALER;
      double thetaDev =
          THETA_SCALE * Math.log(smallestDistanceM) * VISION_CONSTANTS.AUTO_VISION_SCALER
          + THETA_CONSTANT;
      _stdDevs = VecBuilder.fill(xDev, yDev, thetaDev);

    } else if (targetsUsed.size() > 1) {
      double xDev = 0.000634 * smallestDistanceM + 0.00322;
      double yDev = 0.00239 * smallestDistanceM + 0.00121;
      double thetaDev = 0.00201 * smallestDistanceM + 0.145;

      _stdDevs = VecBuilder.fill(xDev, yDev, thetaDev);
    } else {
      double xDev = X_DISTANCE_SCALE * Math.exp(X_DISTANCE_POW * smallestDistanceM)
          * closestTarget.getPoseAmbiguity();
      double yDev = Y_DISTANCE_SCALE * Math.exp(Y_DISTANCE_POW * smallestDistanceM)
          * closestTarget.getPoseAmbiguity();
      double thetaDev = THETA_SCALE * Math.log(smallestDistanceM) + THETA_CONSTANT;
      _stdDevs = VecBuilder.fill(xDev, yDev, thetaDev);
    }
  }

  public double updateCenterOffsetX(int[] tagID, double x_resolution, int offset) {
    var result = _camera.getLatestResult();
    if (result == null) {
      //   _lastTimestamp = 0.0;
      return Double.MAX_VALUE;
    }
    for (PhotonTrackedTarget target : result.getTargets()) {
      for (int id : tagID) {
        if (id == target.getFiducialId()) {
          List<TargetCorner> targetCorners = target.getDetectedCorners();
          _lastTimestamp = result.getTimestampSeconds();
          return (x_resolution / 2.0 + offset) - targetCorners.get(0).x
              - ((targetCorners.get(0).x + targetCorners.get(2).x) / 2.0);
        }
      }
    }
    return Double.MAX_VALUE;
  }

  public int getClosestId() {
    return _closestId;
  }

  public Pose3d getLatestPose() {
    return _latestPose;
  }

  public static double getLatestTimestamp() {
    return _lastTimestamp;
  }

  public Matrix<N3, N1> getLatestStdDevs() {
    return _stdDevs;
  }

  public Matrix<N3, N1> getMaxDevs() {
    return _maxDevs;
  }

  public void setMode(PipelineIndex mode) {
    if (_camera.getPipelineIndex() != mode.index) {
      _camera.setPipelineIndex(mode.index);
    }
  }

  public boolean isMode(PipelineIndex mode) {
    return _camera.getPipelineIndex() == mode.index;
  }

  protected void configShuffleboard() {
    ShuffleboardTab drivetrainTab = Shuffleboard.getTab("Drivetrain");
    String name = "Photon " + _cameraName;
    ShuffleboardLayout photonLayout = drivetrainTab.getLayout(name, BuiltInLayouts.kGrid)
                                          .withSize(2, 3)
                                          .withPosition(2 * _columnCounter++, 0);
    _photonDistanceEntry =
        photonLayout.add(name + " Distance", 0.0).withSize(2, 1).withPosition(0, 0).getEntry();
    _photonYawEntry =
        photonLayout.add(name + " Yaw", 0.0).withSize(2, 1).withPosition(0, 1).getEntry();
    _photonPitchEntry =
        photonLayout.add(name + " Pitch", 0.0).withSize(2, 1).withPosition(0, 2).getEntry();
  }

  protected void updateShuffleboard(double yaw, double distance, double pitch) {
    _photonPitchEntry.setDouble(pitch);
    _photonYawEntry.setDouble(yaw);
    _photonDistanceEntry.setDouble(distance);
  }
  public Pose2d getPiecePose() {
    if (_camera.getPipelineIndex() != PipelineIndex.COLOR.index) {
      setMode(PipelineIndex.COLOR);
    }
    var result = _camera.getLatestResult();
    if (result == null || result.getTargets().size() == 0) {
      // _lastTimestamp = 0.0;
      return null;
    }
    // filter piece(s) by area -> largest area
    PhotonTrackedTarget largestPiece = result.getTargets().get(0);
    for (PhotonTrackedTarget piece : result.getTargets()) {
      if (piece.getArea() > largestPiece.getArea()) {
        largestPiece = piece;
      }
    }
    // find y pixel center
    List<TargetCorner> pieceCorners = largestPiece.getMinAreaRectCorners();
    int yIndex = 0;
    while (yIndex < pieceCorners.size()
           && pieceCorners.get(yIndex + 1).y - pieceCorners.get(yIndex).y == 0) {
      yIndex++;
    }
    double yPixWidth = Math.abs(pieceCorners.get(yIndex + 1).y - pieceCorners.get(yIndex).y);
    double yPixCenter = (pieceCorners.get(yIndex).y < pieceCorners.get(yIndex + 1).y)
        ? pieceCorners.get(yIndex).y + yPixWidth / 2
        : pieceCorners.get(yIndex + 1).y + yPixWidth / 2;
    double forwardDist = 261.807 + (-1.53266 * yPixCenter) + (0.00328175 * Math.pow(yPixCenter, 2))
        + (-0.00000233822 * Math.pow(yPixCenter, 3));
    double largestPieceYaw = largestPiece.getYaw();
    double angleDeg = -4.14 + 1.37 * largestPieceYaw + 0.00383 * Math.pow(largestPieceYaw, 2);

    double meterForwardDist = Units.inchesToMeters(forwardDist);
    if (meterForwardDist <= 0) {
      meterForwardDist = -meterForwardDist;
    }
    if (meterForwardDist > 2.5 || Math.abs(angleDeg) > 36) {
      return null;
    }
    // reflects the largestPiece to the back
    return new Pose2d(
        -(meterForwardDist + 0.15),
        meterForwardDist * Math.tan(Units.degreesToRadians(angleDeg)),
        Rotation2d.fromDegrees(angleDeg)
    );
  }
}
