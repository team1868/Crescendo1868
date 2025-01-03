package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotAltModes;
import frc.robot.constants.enums.CameraSets;
import frc.robot.constants.enums.PipelineIndex;
import frc.robot.constants.enums.VisionModes;
import frc.robot.parsers.VisionParser;
import frc.robot.subsystems.minor.TagalongVisionController;
import frc.robot.utils.FileUtils;
import java.io.File;
import tagalong.controls.PIDSGVAConstants;

public class MultiCameraController {
  public static int _primaryCam = 0, _secondaryCam = 0;

  public static enum VisionState { SINGLE, PRIMARY_2PLUS, SECONDARY_2PLUS }

  public VisionState _currentVisionState = VisionState.SINGLE;

  // TODO: Move to current robot configurations
  public boolean _isVisionMode; // false for on the fly disablement
  public final boolean _isVisionConfigured; // based on configuration
  public final VisionParser _visionParser;
  public final PIDSGVAConstants _visionPIDConstants;

  private FieldObject2d[] _visionFieldPoses = new FieldObject2d[VisionParser.MAX_NUM_CAMERAS];
  private SwerveDrivePoseEstimator _robotPoseEstimator;
  private SwerveDrivePoseEstimator _visionSwervePoseEstimator;
  private FieldObject2d _pieceFieldPose;
  private final AprilTagFieldLayout _aprilTagFieldLayout;

  private TagalongVisionController[] _cameras =
      new TagalongVisionController[VisionParser.MAX_NUM_CAMERAS];

  /* -------- Mode related -------- */
  private VisionModes _curMode;
  private final VisionModes _defaultMode = VisionModes.OFF;

  public MultiCameraController(
      String filePath,
      Field2d field,
      SwerveDrivePoseEstimator robotPoseEstimator,
      SwerveDrivePoseEstimator visionSwervePoseEstimator,
      CameraSets set
  ) {
    this(
        filePath == null ? null : new VisionParser(Filesystem.getDeployDirectory(), filePath, set),
        field,
        robotPoseEstimator,
        visionSwervePoseEstimator
    );
  }

  public MultiCameraController(
      VisionParser visionParser,
      Field2d field,
      SwerveDrivePoseEstimator robotPoseEstimator,
      SwerveDrivePoseEstimator visionSwervePoseEstimator
  ) {
    _isVisionConfigured = visionParser != null;
    _isVisionMode = _isVisionConfigured;
    _visionParser = visionParser;
    _primaryCam = (_visionParser == null || _visionParser._setup == null)
        ? 0
        : _visionParser._setup.bluePrimary;
    _secondaryCam = (_visionParser == null || _visionParser._setup == null)
        ? 0
        : _visionParser._setup.redPrimary;

    if (!_isVisionConfigured) {
      _aprilTagFieldLayout = null;
      _curMode = VisionModes.OFF;
      _visionPIDConstants = null;
      return;
    }
    _robotPoseEstimator = robotPoseEstimator;
    _visionSwervePoseEstimator = visionSwervePoseEstimator;
    _curMode = VisionModes.PREMATCH;

    _visionPIDConstants = _visionParser.cameraVisionConf.visionPID.getPIDSGVAConstants();

    AprilTagFieldLayout layout = null;
    try {
      FileUtils.checkForFile(new File(Constants.CField._visionFieldFile));
      layout = new AprilTagFieldLayout(Constants.CField._visionFieldFile);
    } catch (Exception err) {
      System.err.println(err);
      System.exit(1);
    } finally {
      _aprilTagFieldLayout = layout;
    }

    for (int id : _visionParser._setup.ids) {
      _cameras[id] = new TagalongVisionController(
          _visionParser.cameraConfs[id].cameraName,
          _visionParser.getRobotToCamera(id),
          _aprilTagFieldLayout,
          _visionParser.validTagMap
      );
      _visionFieldPoses[id] =
          field.getObject("Vision Pose " + _visionParser.cameraConfs[id].fieldObjectName);
      _pieceFieldPose = field.getObject("Vision Piece Pose");
    }

    configShuffleboard();
  }

  public int updateOdom(boolean isRedAlliance) {
    if (_isVisionMode) {
      int prim, second;
      switch (_curMode) {
        case PREMATCH:
          _cameras[_primaryCam].updateVisionPose3d(_cameras[_primaryCam].getLatestPose());
          addVisionMeasurement(_primaryCam);
          break;
        case SHOOTING:
          prim = _cameras[_primaryCam].isMode(PipelineIndex.APRILTAG_3D) ? updateOdom(_primaryCam)
                                                                         : -5;
          second = _cameras[_secondaryCam].isMode(PipelineIndex.APRILTAG_3D)
              ? updateOdom(_secondaryCam)
              : -5;
          if (prim > 1) {
            addVisionMeasurement(_primaryCam);
            _currentVisionState = VisionState.PRIMARY_2PLUS;
            // System.out.println("Primary cam");
          } else if (second > 1) {
            addVisionMeasurement(_secondaryCam);
            _currentVisionState = VisionState.SECONDARY_2PLUS;
          }
          break;
        case AUTO:
          _currentVisionState = VisionState.PRIMARY_2PLUS;
          prim = _cameras[_primaryCam].isMode(PipelineIndex.APRILTAG_3D) ? updateOdom(_primaryCam)
                                                                         : -5;
          if (prim > 1) {
            addVisionMeasurement(_primaryCam);
          }
        case TELEOP:
        case CLIMB:
        case AMP:
        case HP_INTAKE:
        case GROUND_INTAKE:
          prim = _cameras[_primaryCam].isMode(PipelineIndex.APRILTAG_3D) ? updateOdom(_primaryCam)
                                                                         : -5;
          second = _cameras[_secondaryCam].isMode(PipelineIndex.APRILTAG_3D)
              ? updateOdom(_secondaryCam)
              : -5;
          if (prim <= 0 && second <= 0) {
            return 0;
          } else if (prim > 1) {
            addVisionMeasurement(_primaryCam);
            _currentVisionState = VisionState.PRIMARY_2PLUS;
          } else if (second > 1) {
            addVisionMeasurement(_secondaryCam);
            _currentVisionState = VisionState.SECONDARY_2PLUS;
          }
          int _max = prim > second ? prim : second;
          return _max;
        case OFF:
        default:
          break;
      }
    }
    return 0;
  }

  protected void configShuffleboard() {}

  public void updateOdom() {
    updateOdom(_primaryCam);
  }

  private void addVisionMeasurement(int id) {
    _robotPoseEstimator.addVisionMeasurement(
        _cameras[id].getLatestPose().toPose2d(),
        TagalongVisionController.getLatestTimestamp(),
        _cameras[id].getLatestStdDevs()
    );
    _visionSwervePoseEstimator.addVisionMeasurement(
        _cameras[id].getLatestPose().toPose2d(),
        TagalongVisionController.getLatestTimestamp(),
        _cameras[id].getLatestStdDevs()
    );
  }

  private int updateOdom(int id) {
    double lastTimestamp = TagalongVisionController.getLatestTimestamp();
    _cameras[id].updateVisionPose3d(_cameras[id].getLatestPose());
    if (lastTimestamp < TagalongVisionController.getLatestTimestamp()) {
      return _cameras[id]._lastTargetCount;
    }
    return -4; // timestamp miss
  }

  public Pose2d getVisionPose(int id) {
    if (_isVisionMode && _visionParser.isValidId(id)) {
      return _cameras[id].getLatestPose().toPose2d();
    }
    return new Pose2d();
  }

  public Pose2d getVisionPose() {
    if (_isVisionMode) {
      return getVisionPose(_primaryCam);
    }
    return new Pose2d();
  }

  public void setPipeline(PipelineIndex index) {
    if (_isVisionMode) {
      _cameras[_primaryCam].setMode(index);
      _cameras[_secondaryCam].setMode(index);
    }
  }

  public void setColorPipeline(PipelineIndex index) {
    if (_isVisionMode && _visionParser._setup.hasCamera2()) {
      _cameras[2].setMode(index);
    }
  }

  public void setVisionMode(VisionModes visionMode) {
    if (_isVisionMode) {
      _curMode = visionMode;
      if (_visionParser._setup.hasCamera0()) {
        _cameras[0].setMode(_curMode.primaryPipeline);
      }
      if (_visionParser._setup.hasCamera1()) {
        _cameras[1].setMode(_curMode.primaryPipeline);
      }
      if (_visionParser._setup.hasCamera2()) {
        _cameras[2].setMode(_curMode.colorPipeline);
      }
    }
  }

  public VisionModes getVisionMode() {
    return _curMode;
  }

  protected void updateShuffleboard() {
    if (_isVisionMode) {
      for (int id : _visionParser._setup.ids) {
        _visionFieldPoses[id].setPose(getVisionPose(id));
      }
    }
  }

  public double getCenterOffsetX(VisionModes visionMode, boolean isRedAlliance) {
    if (_isVisionMode && _visionParser._setup.hasCamera0()) {
      int id = _primaryCam;
      double lastTimestamp = TagalongVisionController.getLatestTimestamp();
      double centerOffsetX = _cameras[id].updateCenterOffsetX(
          isRedAlliance ? visionMode.redTagId : visionMode.blueTagId,
          _visionParser.cameraConfs[id].x_resolution,
          0
      );
      if (lastTimestamp < TagalongVisionController.getLatestTimestamp()) {
        if (centerOffsetX < VisionParser.PIX_2D_TOLERANCE
            && centerOffsetX > -VisionParser.PIX_2D_TOLERANCE) {
          return 0.0;
        }
        return centerOffsetX;
      }
    }
    return Double.POSITIVE_INFINITY;
  }

  public void disableVision(boolean disable) {
    _isVisionMode = disable && _isVisionConfigured;
  }

  public Pose2d getPiecePose(Pose2d drivetrainLoc) {
    if (_isVisionMode && _curMode == VisionModes.GROUND_INTAKE) {
      Pose2d piecePose = _cameras[2].getPiecePose();
      if (RobotAltModes.isPoseTuning) {
        _pieceFieldPose.setPose(piecePose.relativeTo(drivetrainLoc));
      }
      return piecePose;
    }
    return null;
  }
}
