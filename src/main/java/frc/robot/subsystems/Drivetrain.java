package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveControlRequestParameters;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Control;
import frc.robot.constants.RobotAltModes;
import frc.robot.constants.enums.DriveModes;
import frc.robot.constants.enums.ScoreTargets;
import frc.robot.constants.enums.VisionModes;
import frc.robot.parsers.SwerveParser;
import frc.robot.parsers.json.SwerveDriveControlJson;
import frc.robot.parsers.json.SwerveDriveJson;
import frc.robot.utils.GyroIO;
import frc.robot.utils.GyroIOInputsAutoLogged;
import frc.robot.utils.GyroIOPigeon2;
import frc.robot.utils.InputUtils;
import frc.robot.utils.LoopTimer;
import frc.robot.utils.SwerveModuleSetpointGenerator;
import frc.robot.utils.SwerveSetpoint;
import frc.robot.utils.TagalongSwerveModuleBase;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;
import tagalong.math.AlgebraicUtils;
import tagalong.subsystems.TagalongSubsystemBase;
import tagalong.subsystems.micro.Microsystem;

public class Drivetrain extends TagalongSubsystemBase {
  public static final double ALLOWED_SCRUB = 0.25;
  public static final Controlboard _controlboard = Controlboard.get();
  private final boolean USE_POSE_ESTIMATION_ANGLE = false; // change if true?

  /* --- Sensors, motors, and hardware --- */
  private Pigeon2 _gyro;
  private Pigeon2Configuration _gyroConfigs;

  /* -------- Logging: utilities and configs -------- */
  private final GyroIO _gyroIO;
  private final GyroIOInputsAutoLogged _gyroInputs;

  // Thread odometry resources
  private final OdometryThread _odometryThread;
  protected final ReadWriteLock _odometryLock = new ReentrantReadWriteLock();
  private double _averageOdometryLoopTime = 0.0;

  // 604 antiscrub utils and odometry
  private final SwerveModuleSetpointGenerator _setpointGenerator;
  /* Module Slew Rate Limits */
  // Max module acceleration can be high since drive wheels can be backdriven.
  public static final double _maxModuleAcceleration = 1000.0; // m/s/s
  public static final double _maxModuleSteeringRate = 4.0 * Math.PI; // rad/s/s
  public SwerveSetpoint _previousModuleSetpoint;
  public SwerveModuleState[] _states;

  public final double _translationalToleranceM;
  public final Rotation2d _rotationalToleranceRot;

  public final Constraints _translationalLimitsM;
  public final double _angularMaxVeloRad;
  public final double _angularMaxAccelRad;

  private final TagalongSwerveModuleBase[] _modules;

  /* --- State and Physical Property variables --- */
  private ChassisSpeeds _chassisSpeeds = new ChassisSpeeds();
  // Initialize with real values regardless of canbus state to ensure good launch
  // to pose
  // estimator
  private SwerveModulePosition[] _modulePositions;

  /* --- Odometry Utils --- */
  // TODO: move to configurations
  private Translation2d[] _swerveModuleLocations;
  private final edu.wpi.first.math.kinematics.SwerveDriveKinematics _swerveKinematics;

  SwerveDrivePoseEstimator _robotPoseEstimator;
  SwerveDrivePoseEstimator _visionSwervePoseEstimator;
  private MultiCameraController _multiCameraController;

  private int _maxTargetsSeen;

  SwerveDriveOdometry _odom;

  // weird stuff for threaded implementation
  protected SwerveControlRequestParameters _requestParameters =
      new SwerveControlRequestParameters();
  protected Rotation2d _fieldRelativeOffset;
  protected SwerveRequest _requestToApply = new SwerveRequest.Idle();

  /**
   * Plain-Old-Data class holding the state of the swerve drivetrain.
   * This encapsulates most data that is relevant for telemetry or
   * decision-making from the Swerve Drive.
   */
  public class SwerveDriveState {
    public int _successfulDaqs;
    public int _failedDaqs;
    public Pose2d _estimatedPose;
    public Rotation2d _yaw;
    public SwerveModuleState[] _moduleStates;
    public double _odometryPeriod;
  }

  protected Consumer<SwerveDriveState> _telemetryFunction = null;
  protected SwerveDriveState _cachedState = new SwerveDriveState();

  /* --- Game State variables --- */
  private Field2d _field;
  private boolean _isAuto = false;
  private Translation2d _faceTargetGoal = new Translation2d();

  /* --- Control Utils --- */
  private ProfiledPIDController _xController;
  private ProfiledPIDController _yController;
  private ProfiledPIDController _angleController;
  private ProfiledPIDController _visionCenterOffsetController;
  private SlewRateLimiter _xSlewRateFilter;
  private SlewRateLimiter _ySlewRateFilter;
  private SlewRateLimiter _angleSlewRateFilter;

  /* --- Simulation resources and variables --- */
  private Pose2d _simPose = new Pose2d();

  /* --- Logging variables --- */
  private final double[] _desiredSwerveStates;
  private final double[] _currentSwerveStates;

  /* --- Shuffleboard entries --- */
  private SendableChooser<ScoreTargets> _staticTargetChooser = new SendableChooser<ScoreTargets>();
  private GenericEntry _desiredSpeed, _actualSpeed;
  private GenericEntry _actualSpeedX, _actualSpeedY, _actualSpeedTheta;
  private GenericEntry _desiredRobotTheta, _actualRobotTheta, _errorRobotTheta;

  private GenericPublisher _xPoseError, _yPoseError, _thetaPoseError;
  private GenericEntry _usingPoseTuning;

  private GenericEntry _anglePFac, _angleIFac, _angleDFac;
  private GenericEntry _xPFac, _xIFac, _xDFac;
  private GenericEntry _yPFac, _yIFac, _yDFac;

  private GenericEntry _steerTargetAngle, _steerErrorAngle;
  private GenericEntry _steerAnglePFac, _steerAngleIFac, _steerAngleDFac;
  private GenericEntry _steerPFac, _steerIFac, _steerDFac;

  public GenericPublisher _targetPositionEntry, _targetVelocityEntry;
  public GenericPublisher _curPositionEntry, _curVelocityEntry;

  private Pose2d TARGET_RELATIVE_POSE = new Pose2d(0.0, 1.0, Rotation2d.fromDegrees(0.0));
  public final SwerveParser _swerveParser;
  public final SwerveDriveJson _swerveConf;
  public final SwerveDriveControlJson _swerveControl;

  public Drivetrain(String filePath, Field2d field) {
    this(
        filePath == null ? null : new SwerveParser(Filesystem.getDeployDirectory(), filePath), field
    );
  }

  public Drivetrain(SwerveParser parser, Field2d field) {
    super(parser);
    _swerveParser = parser;

    if (parser == null) {
      _swerveConf = null;
      _swerveControl = null;
      _gyroIO = null;
      _gyroInputs = null;
      _odometryThread = null;
      _swerveKinematics = null;
      _desiredSwerveStates = new double[0];
      _currentSwerveStates = new double[0];
      _modules = new TagalongSwerveModuleBase[0];
      _setpointGenerator = null;
      _translationalToleranceM = 0.0;
      _rotationalToleranceRot = new Rotation2d();
      _translationalLimitsM = new TrapezoidProfile.Constraints(0.0, 0.0);
      _angularMaxVeloRad = 0.0;
      _angularMaxAccelRad = 0.0;

      return;
    }
    _swerveConf = _swerveParser.swerveConf;
    _swerveControl = _swerveConf.drivetrainControl;
    _field = field;

    // Setup Gyro and IO
    _gyro = _swerveConf.IMU.getPigeon2();
    _gyroConfigs = new Pigeon2Configuration();

    // Get modules from configs
    _modules = _swerveParser.getSwerveModules();
    _modulePositions = new SwerveModulePosition[_swerveParser._numModules];
    _states = new SwerveModuleState[_swerveParser._numModules];
    _swerveModuleLocations = _swerveParser.getSwerveModuleLocations();
    for (int i = 0; i < _swerveParser._numModules; i++) {
      _modulePositions[i] = _modules[i].getPosition();
      _states[i] = _modules[i].getState();
    }

    // Fetch and cache swerve constants
    _translationalToleranceM = _swerveControl.translationalTolerance.getLengthM();
    _rotationalToleranceRot = _swerveControl.rotationalTolerance.getDistRotation();
    _translationalLimitsM = _swerveControl.defaultLimits.translation.getLimitsM();
    _angularMaxVeloRad = _swerveControl.defaultLimits.angular.getLimitsRad().maxVelocity;
    _angularMaxAccelRad = _swerveControl.defaultLimits.angular.getLimitsRad().maxAcceleration;

    // THIS HAS TO GO BEFORE THE MULTICAMERA INITIALIZATION
    _swerveKinematics = new SwerveDriveKinematics(_swerveModuleLocations);
    _robotPoseEstimator = new SwerveDrivePoseEstimator(
        _swerveKinematics, Rotation2d.fromDegrees(0.0), _modulePositions, new Pose2d()
    );
    _visionSwervePoseEstimator = new SwerveDrivePoseEstimator(
        _swerveKinematics, Rotation2d.fromDegrees(0.0), _modulePositions, new Pose2d()
    );

    // Initialize all cameras
    _multiCameraController = new MultiCameraController(
        Constants.CRobot._vision,
        _field,
        _robotPoseEstimator,
        _visionSwervePoseEstimator,
        Constants.CRobot._cameraSets
    );

    // logging utils
    _desiredSwerveStates = new double[2 * _swerveParser._numModules];
    _currentSwerveStates = new double[2 * _swerveParser._numModules];

    /* --- Control Utils --- */
    var xy = _swerveControl.translationalControl.getPIDSGVAConstants();
    _xController =
        xy.getProfiledController(_swerveControl.defaultTrapezoidalLimits.translation.getLimitsM());
    _yController =
        xy.getProfiledController(_swerveControl.defaultTrapezoidalLimits.translation.getLimitsM());
    _angleController = _swerveControl.rotationalControl.getPIDSGVAConstants().getProfiledController(
        _swerveControl.defaultTrapezoidalLimits.angular.getLimitsRad()
    );
    // TODO: set limits
    if (_multiCameraController._visionPIDConstants != null) {
      _visionCenterOffsetController =
          _multiCameraController._visionPIDConstants.getProfiledController(
              _swerveControl.defaultTrapezoidalLimits.translation.getLimitsM()
          );
    }

    resetDefaultTolerance();
    _angleController.enableContinuousInput(0.0, Units.degreesToRadians(360.0));
    _angleController.setTolerance(_rotationalToleranceRot.getRadians());

    _xSlewRateFilter = _swerveControl.getTranslationalSlewRateLimiter();
    _ySlewRateFilter = _swerveControl.getTranslationalSlewRateLimiter();
    _angleSlewRateFilter = _swerveControl.getAngularSlewRateLimiter();

    // TODO: move module accel/deccel to config file
    _setpointGenerator = new SwerveModuleSetpointGenerator(
        _swerveKinematics,
        _swerveParser._theoreticalMaxWheelSpeedMPS,
        _maxModuleAcceleration,
        _maxModuleSteeringRate
    );
    _previousModuleSetpoint = new SwerveSetpoint(_chassisSpeeds, _states);

    // because ryan introduced a bug, need to reintro to the configs RYAN
    // Also need sto be taken from some config file
    if (Robot.isReal()) {
      int counter = 0;
      while (!checkInitStatus()) {
        System.out.println("DRIVETRAIN Check Init Status : " + counter++);
      }
    }
    _gyroIO = new GyroIOPigeon2(this);
    _gyroInputs = new GyroIOInputsAutoLogged();

    // _yaw.setUpdateFrequency(250.0);
    // _yawVelocity.setUpdateFrequency(250.0);
    // _gyro.optimizeBusUtilization();
    // _yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(_gyro,
    // _gyro.getYaw());

    zeroGyro();
    configShuffleboard();

    if (RobotAltModes.isPoseTuning) {
      // _xPoseError;
      // _yPoseEntry;
      // _thetaPoseError;
      // _usingPoseTuning;
    }

    if (RobotAltModes.isPIDTuningMode) {
      // _anglePFac;
      // _angleIFac;
      // _angleDFac;
      // _xPFac;
      // _xIFac;
      // _xDFac;
      // _yPFac;
      // _yIFac;
      // _yDFac;
      // _steerTargetAngle
      // _steerAnglePFac;
      // _steerAngleIFac;
      // _steerAngleDFac;
      // _steerPFac;
      // _steerIFac;
      // _steerDFac;
    }

    _fieldRelativeOffset = new Rotation2d();
    _odometryThread = new OdometryThread(_modules);
    if (RobotAltModes.kEnableThreadedOdometry) {
      _odometryThread.start();
    }
    LoopTimer.markEvent(" Drivetrain Initialization Complete: ");
  }

  @Override
  public void periodic() {
    if (_isSubsystemDisabled) {
      return;
    }

    LoopTimer.markLoopStart();

    if (RobotAltModes.kEnableThreadedOdometry) {
      double averageOdometryLoopTime = _cachedState._odometryPeriod;
      // System.out.println("Average odometry loop time: " + averageOdometryLoopTime);
    } else {
      // get module positions ensures the order of data begin collected
      try {
        var modulePositions = getModulePositions();
        _odometryLock.writeLock().lock();
        _robotPoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getYaw(), modulePositions);
        _visionSwervePoseEstimator.updateWithTime(
            Timer.getFPGATimestamp(), getYaw(), modulePositions
        );
      } finally {
        _odometryLock.writeLock().unlock();
      }
    }

    LoopTimer.markEvent(" PoseEstimator ");

    try {
      _odometryLock.writeLock().lock();
      _maxTargetsSeen = _multiCameraController.updateOdom(Controlboard.isRedAlliance());
    } finally {
      _odometryLock.writeLock().unlock();
    }
    LoopTimer.markEvent(" Vision ");

    updateShuffleboard();

    if (!RobotAltModes.kEnableThreadedOdometry) {
      // TODO: call an update function and pre-construct the array
      _chassisSpeeds = _swerveKinematics.toChassisSpeeds(
          _modules[0].getState(),
          _modules[1].getState(),
          _modules[2].getState(),
          _modules[3].getState()
      );
    } else {
      // System.out. println(getPose());
    }

    // Logging
    _gyroIO.updateInputs(_gyroInputs);
    Logger.processInputs("Drive/Gyro", _gyroInputs);
    for (var module : _modules) {
      module.updateInputs();
      module.periodic();
    }
    // System.out.println("yaw " + getYaw().getRotations() * 360.0);

    Logger.processInputs("Drive/Gyro", _gyroInputs);

    LoopTimer.markCompletion(" Drivetrain Shuffleboard ", "\n Total Drivetrain ");

    // System.out.println(getYaw().getDegrees());
  }

  public SwerveModulePosition[] getModulePositions() {
    if (!_isSubsystemDisabled) {
      for (int i = 0; i < _swerveParser._numModules; i++) {
        _modulePositions[i] = _modules[i].getPosition();
      }
    }
    return _modulePositions;
  }

  public void setAutoMode(boolean enable) {
    _isAuto = enable;
  }

  @Override
  protected void configShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("SwerveModules");
    _steerTargetAngle = tab.add("target angle", 0.0).getEntry();

    ShuffleboardTab drivetrainTab = Shuffleboard.getTab("Drivetrain");

    if (RobotAltModes.isPoseTuning) {
      ShuffleboardLayout poseLayout = drivetrainTab.getLayout("Go To Pose", BuiltInLayouts.kGrid)
                                          .withSize(1, 2)
                                          .withPosition(7, 0);

      _usingPoseTuning = poseLayout.add("using pose tuning", false).withPosition(0, 0).getEntry();
      _xPoseError = poseLayout.add("x Pose Error", 0.0).withPosition(0, 1).getEntry();
      _yPoseError = poseLayout.add("y Pose Error", 0.0).withPosition(0, 2).getEntry();
      _thetaPoseError = poseLayout.add("theta Pose Error", 0.0).withPosition(0, 3).getEntry();
    }

    if (RobotAltModes.isPIDTuningMode) {
      int PIDCOL = 3;
      int PIDROW = 1;

      _errorRobotTheta = drivetrainTab.add("Error Angle", 0.0).getEntry();
      ShuffleboardLayout anglePIDLayout = drivetrainTab.getLayout("Angle PID", BuiltInLayouts.kGrid)
                                              .withSize(1, 3)
                                              .withPosition(PIDCOL, PIDROW);
      _anglePFac = anglePIDLayout.add("P", _angleController.getP()).withPosition(0, 0).getEntry();
      _angleIFac = anglePIDLayout.add("I", _angleController.getI()).withPosition(0, 1).getEntry();
      _angleDFac = anglePIDLayout.add("D", _angleController.getD()).withPosition(0, 2).getEntry();

      ShuffleboardLayout xLayout = drivetrainTab.getLayout("X PID", BuiltInLayouts.kGrid)
                                       .withSize(1, 3)
                                       .withPosition(PIDCOL + 1, PIDROW);
      _xPFac = xLayout.add("P", _xController.getP()).withPosition(0, 0).getEntry();
      _xIFac = xLayout.add("I", _xController.getI()).withPosition(0, 1).getEntry();
      _xDFac = xLayout.add("D", _xController.getD()).withPosition(0, 2).getEntry();

      ShuffleboardLayout yLayout = drivetrainTab.getLayout("Y PID", BuiltInLayouts.kGrid)
                                       .withSize(1, 3)
                                       .withPosition(PIDCOL + 2, PIDROW);
      _yPFac = yLayout.add("P", _yController.getP()).withPosition(0, 0).getEntry();
      _yIFac = yLayout.add("I", _yController.getI()).withPosition(0, 1).getEntry();
      _yDFac = yLayout.add("D", _yController.getD()).withPosition(0, 2).getEntry();
    }

    ShuffleboardTab controlboardTab = Shuffleboard.getTab("Competition HUD");
    controlboardTab.add("Field", _field).withSize(11, 5).withPosition(1, 1);

    if (RobotAltModes.isSOTFTuningMode) {
      ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning tab");
      ShuffleboardLayout faceTargetLayout =
          tuningTab.getLayout("drivetrain", BuiltInLayouts.kGrid)
              .withSize(3, 4)
              .withPosition(2 * Microsystem._tuningTabCounter++, 0);
      _targetPositionEntry =
          faceTargetLayout.add("Face Target Position", 0.0).withPosition(2, 0).getEntry();
      _targetVelocityEntry =
          faceTargetLayout.add("Face Target Velocity", 0.0).withPosition(3, 0).getEntry();
      _curPositionEntry =
          faceTargetLayout.add("Face Current Position", 0.0).withPosition(4, 0).getEntry();
      _curVelocityEntry =
          faceTargetLayout.add("Face Current Velocity", 0.0).withPosition(5, 0).getEntry();
    }
  }

  @Override
  protected void updateShuffleboard() {
    for (var module : _modules) {
      module.updateShuffleboard();
    }

    // _actualAngle->SetDouble(ToAbsoluteAngle(GetYaw()));

    // _actualSpeedX->SetDouble(_chassisSpeeds.vx.value());
    // _actualSpeedY->SetDouble(_chassisSpeeds.vy.value());
    // _actualAngularVelocity->SetDouble(_chassisSpeeds.omega.value());

    _multiCameraController.updateShuffleboard();

    var pose = getPose();
    _field.setRobotPose(pose.getX(), pose.getY(), pose.getRotation());

    if (RobotAltModes.isPIDTuningMode) {
      _errorRobotTheta.setDouble(Units.degreesToRadians(_angleController.getPositionError()));
    }
  }

  @Override
  public void simulationInit() {
    if (!Robot.isReal() && !_isSubsystemDisabled) {
      for (var module : _modules) module.simulationInit();
    }
  }

  @Override
  public void simulationPeriodic() {
    if (!Robot.isReal() && !_isSubsystemDisabled) {
      for (var module : _modules) module.simulationPeriodic();

      _simPose = _simPose.transformBy(new Transform2d(
          new Translation2d(
              _chassisSpeeds.vxMetersPerSecond * Constants.LOOP_PERIOD_S,
              _chassisSpeeds.vyMetersPerSecond * Constants.LOOP_PERIOD_S
          ),
          Rotation2d.fromRadians(_chassisSpeeds.omegaRadiansPerSecond * Constants.LOOP_PERIOD_S)
      ));

      _gyro.getSimState().setRawYaw(_simPose.getRotation().getDegrees());
    }
  }

  public Rotation2d getYaw() {
    return _isSubsystemDisabled ? new Rotation2d()
                                : Rotation2d.fromDegrees(_gyro.getYaw().getValueAsDouble());
  }

  public Rotation2d getPitch() {
    return _isSubsystemDisabled ? new Rotation2d()
                                : Rotation2d.fromDegrees(_gyro.getPitch().getValueAsDouble());
  }

  public Rotation2d getRoll() {
    return _isSubsystemDisabled ? new Rotation2d()
                                : Rotation2d.fromDegrees(_gyro.getRoll().getValueAsDouble());
  }

  public Pose2d getPose() {
    if (_isSubsystemDisabled) {
      return new Pose2d();
    } else if (RobotAltModes.kEnableThreadedOdometry) {
      try {
        _odometryLock.readLock().lock();
        return _cachedState._estimatedPose;
      } finally {
        _odometryLock.readLock().unlock();
      }
    } else {
      return _robotPoseEstimator.getEstimatedPosition();
    }
  }

  public double[] swerveMeasuredIO() {
    if (!_isSubsystemDisabled) {
      for (int i = 0; i < _swerveParser._numModules; i++) {
        SwerveModuleState moduleState = _modules[i].getState();
        _currentSwerveStates[i * 2] = moduleState.angle.getDegrees();
        _currentSwerveStates[(i * 2) + 1] = moduleState.speedMetersPerSecond;
      }
    }
    return _currentSwerveStates;
  }

  public double[] swerveSetpointsIO() {
    return _desiredSwerveStates;
  }

  private void setDesiredSwerveState(SwerveModuleState[] goalModuleStates) {
    for (int i = 0; i < _swerveParser._numModules; i++) {
      SwerveModuleState state = goalModuleStates[i];
      _desiredSwerveStates[i * 2] = state.angle.getDegrees();
      _desiredSwerveStates[(i * 2) + 1] = state.speedMetersPerSecond;
    }
  }

  private boolean areWheelsAligned(SwerveModuleState[] goalStates) {
    for (int i = 0; i < _swerveParser._numModules; i++) {
      if (!_modules[i].isAlignedTo(goalStates[i]))
        return false;
    }
    return true;
  }

  private boolean areWheelsAligned(SwerveModuleState goalState) {
    for (int i = 0; i < _swerveParser._numModules; i++) {
      if (!_modules[i].isAlignedTo(goalState))
        return false;
    }
    return true;
  }

  private void resetModulesToAbsolute() {
    for (int i = 0; i < _swerveParser._numModules; i++) {
      _modules[i].resetToAbsolute();
      _states[i] = _modules[i].getState();
    }
    _previousModuleSetpoint = new SwerveSetpoint(_chassisSpeeds, _states);
  }

  private void alignModulesToZero() {
    for (var module : _modules) {
      module.alignToZero();
    }
  }

  public void zeroGyro() {
    Pose2d pose = getPose();
    setPose(pose == null ? new Pose2d() : pose, _controlboard.allianceGyroAngle());
  }

  public void zeroPose() {
    setPose(new Pose2d());
  }

  public void autoZeroGyro() {
    setPose(getPose(), getYaw().plus(Rotation2d.fromDegrees(_controlboard.POVZeroOffsetDeg())));
  }

  public void setPose(Pose2d pose, Rotation2d yaw) {
    try {
      _odometryLock.writeLock().lock();
      _gyro.setYaw(yaw.getDegrees());
    } finally {
      _odometryLock.writeLock().unlock();
    }
    setOdometryPose(pose, yaw);
  }

  public void setPose(Pose2d pose) {
    setOdometryPose(pose, getYaw());
  }

  private void setOdometryPose(Pose2d pose, Rotation2d yaw) {
    try {
      _odometryLock.writeLock().lock();
      var newPose = new Pose2d(pose.getTranslation(), yaw);
      _robotPoseEstimator.resetPosition(yaw, _modulePositions, newPose);
      _visionSwervePoseEstimator.resetPosition(yaw, _modulePositions, newPose);
    } finally {
      _odometryLock.writeLock().unlock();
    }
  }

  public boolean motorResetConfig() {
    for (var module : _modules) {
      if (module.motorResetConfig())
        return true;
    }
    return false;
  }

  // CRUCIAL
  // TODO: Depending on april tags seen, return true if valid stage tag for a
  // given alliance is seen
  public boolean seesValidStageTarget() {
    return true;
  }

  // TODO: returns mapped integer id for given valid stage target apriltag seen in
  // seesValidStageTarget;
  public int getStageTargetID() {
    return 1;
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates, double maxSpeed) {
    // Desaturate based of max theoretical or functional rather than current max
    edu.wpi.first.math.kinematics.SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, maxSpeed
    );

    for (var module : _modules) {
      module.setDesiredState(desiredStates[module._moduleNumber], Constants.Control.IS_OPEN_LOOP);
    }

    setDesiredSwerveState(desiredStates);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    setModuleStates(desiredStates, _translationalLimitsM.maxVelocity);
  }

  /**
   * Approximation to correct for for swerve second order dynamics issue. Inspired
   * by 254:
   * https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/subsystems/Drive.java#L325
   * Discussion:
   * https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
   */
  private static ChassisSpeeds correctForDynamics(final ChassisSpeeds originalSpeeds) {
    final Pose2d robotPoseVel = new Pose2d(
        originalSpeeds.vxMetersPerSecond * Constants.LOOP_PERIOD_S,
        originalSpeeds.vyMetersPerSecond * Constants.LOOP_PERIOD_S,
        Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * Constants.LOOP_PERIOD_S)
    );
    final Twist2d twistVel = AlgebraicUtils.log(robotPoseVel);
    return new ChassisSpeeds(
        twistVel.dx / Constants.LOOP_PERIOD_S,
        twistVel.dy / Constants.LOOP_PERIOD_S,
        twistVel.dtheta / Constants.LOOP_PERIOD_S
    );
  }

  public void drive(
      double joystickX,
      double joystickY,
      double compositeXY,
      double joystickTheta,
      DriveModes mode,
      double maxSpeedMPS,
      Rotation2d maxAngularSpeed
  ) {
    switch (mode) {
      case ROBOT_CENTRIC:
        robotCentricDrive(
            InputUtils.scaleJoystickXMPS(joystickX, compositeXY, maxSpeedMPS),
            InputUtils.scaleJoystickYMPS(joystickY, compositeXY, maxSpeedMPS),
            InputUtils.ScaleJoystickThetaRadPS(joystickTheta, maxAngularSpeed),
            maxSpeedMPS
        );
        break;
      case FIELD_RELATIVE:
        fieldRelativeDrive(
            InputUtils.scaleJoystickXMPS(joystickX, compositeXY, maxSpeedMPS),
            InputUtils.scaleJoystickYMPS(joystickY, compositeXY, maxSpeedMPS),
            InputUtils.ScaleJoystickThetaRadPS(joystickTheta, maxAngularSpeed),
            maxSpeedMPS
        );
        break;
      case FIELD_RELATIVE_ROTATION_COMPENSATION_SCALING:
        rotationCompensationFieldRelativeDrive(
            InputUtils.scaleJoystickXMPS(joystickX, compositeXY, 1.0),
            InputUtils.scaleJoystickYMPS(joystickY, compositeXY, 1.0),
            InputUtils.ScaleJoystickThetaRadPS(joystickTheta, 1.0),
            maxSpeedMPS
        );
      case SNAP_TO_ANGLE:
        snapToAngleDrive(
            InputUtils.scaleJoystickXMPS(joystickX, compositeXY, maxSpeedMPS),
            InputUtils.scaleJoystickYMPS(joystickY, compositeXY, maxSpeedMPS),
            maxSpeedMPS,
            maxAngularSpeed.getRadians()
        );
        break;
      case SNAKE:
        snakeDrive(joystickX, joystickY, compositeXY, maxSpeedMPS, maxAngularSpeed.getRadians());
        break;
      case TARGET_RELATIVE:
        targetCentricDrive(
            InputUtils.scaleJoystickXMPS(joystickX, compositeXY, maxSpeedMPS),
            InputUtils.scaleJoystickYMPS(joystickY, compositeXY, maxSpeedMPS),
            InputUtils.ScaleJoystickThetaRadPS(joystickTheta, maxAngularSpeed),
            new Translation2d(Constants.CField.halfXM, Constants.CField.halfYM),
            maxSpeedMPS
        );
        break;
      case CHASE_STATIC_TARGET:
        chaseStaticTargetDrive(maxSpeedMPS);
        break;
      case SLEWING_FIELD_RELATIVE:
        slewingFieldRelativeDrive(
            InputUtils.scaleJoystickXMPS(joystickX, compositeXY, maxSpeedMPS),
            InputUtils.scaleJoystickYMPS(joystickY, compositeXY, maxSpeedMPS),
            InputUtils.ScaleJoystickThetaRadPS(joystickTheta, maxAngularSpeed),
            maxSpeedMPS
        );
        break;
      case FIELD_RELATIVE_SKEW_COMPENSATION:
        fieldRelativeDrive(
            InputUtils.scaleJoystickXMPS(joystickX, compositeXY, maxSpeedMPS),
            InputUtils.scaleJoystickYMPS(joystickY, compositeXY, maxSpeedMPS),
            InputUtils.ScaleJoystickThetaRadPS(joystickTheta, maxAngularSpeed),
            maxSpeedMPS
        );
        break;
      case FACE_TARGET_MODE:
        faceTargetMode(
            InputUtils.scaleJoystickXMPS(joystickX, compositeXY, maxSpeedMPS),
            InputUtils.scaleJoystickYMPS(joystickY, compositeXY, maxSpeedMPS),
            maxSpeedMPS,
            maxAngularSpeed.getRadians()
        );
        break;
      case CHASE_DYNAMIC_TARGET:
        // TODO update this function to use the AprilTag layout
        // ChaseDynamicTargetDrive(FIELD_TO_TARGET, TARGET_RELATIVE_POSE, true,
        // maxSpeed);
        // ChaseDynamicTargetDrive(FIELD_TO_TARGET, TARGET_RELATIVE_POSE, _hasTarget);
        break;
      case FIELD_RELATIVE_ANTISCRUB:
        fieldRelativeAntiscrubDrive(
            InputUtils.scaleJoystickXMPS(joystickX, compositeXY, maxSpeedMPS),
            InputUtils.scaleJoystickYMPS(joystickY, compositeXY, maxSpeedMPS),
            InputUtils.ScaleJoystickThetaRadPS(joystickTheta, maxAngularSpeed),
            Drivetrain.ALLOWED_SCRUB,
            maxSpeedMPS
        );
        break;
      default:
        // ERROR
        // throw, error message, or default behavior
        // return;
        break;
    }
  }

  public void drive(double joystickX, double joystickY, double joystickTheta) {
    drive(
        joystickX,
        joystickY,
        Math.hypot(joystickX, joystickY),
        joystickTheta,
        DriveModes.FIELD_RELATIVE_ANTISCRUB
    );
  }

  public void drive(double joystickX, double joystickY, double joystickTheta, DriveModes mode) {
    drive(joystickX, joystickY, Math.hypot(joystickX, joystickY), joystickTheta, mode);
  }

  public void drive(double joystickX, double joystickY, double compositeXY, double joystickTheta) {
    drive(joystickX, joystickY, compositeXY, joystickTheta, DriveModes.FIELD_RELATIVE_ANTISCRUB);
  }

  public void drive(
      double joystickX, double joystickY, double compositeXY, double joystickTheta, DriveModes mode
  ) {
    drive(
        joystickX, joystickY, compositeXY, joystickTheta, mode, _translationalLimitsM.maxVelocity
    );
  }

  public void drive(
      double joystickX,
      double joystickY,
      double compositeXY,
      double joystickTheta,
      DriveModes mode,
      double maxSpeedMPS
  ) {
    drive(
        joystickX,
        joystickY,
        compositeXY,
        joystickTheta,
        mode,
        maxSpeedMPS,
        new Rotation2d(_angularMaxVeloRad)
    );
  }

  // individual drive functions
  public void robotCentricDrive(
      double xTranslationalMPS, double yTranslationalMPS, double rotationRadPS, double maxSpeed
  ) {
    robotCentricDrive(
        new ChassisSpeeds(xTranslationalMPS, yTranslationalMPS, rotationRadPS), maxSpeed
    );
  }

  public void robotCentricDrive(
      double xTranslationalMPS, double yTranslationalMPS, double rotationRadPS
  ) {
    robotCentricDrive(
        xTranslationalMPS, yTranslationalMPS, rotationRadPS, _translationalLimitsM.maxVelocity
    );
  }

  public void robotCentricDrive(ChassisSpeeds speeds) {
    robotCentricDrive(speeds, _translationalLimitsM.maxVelocity);
  }

  public void robotCentricDrive(ChassisSpeeds speeds, double maxSpeed) {
    if (_isSubsystemDisabled) {
      return;
    }

    SwerveModuleState[] goalModuleStates = _swerveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(goalModuleStates, maxSpeed);
  }

  public boolean isOversaturated(SwerveModuleState[] states) {
    for (SwerveModuleState state : states) {
      if (state.speedMetersPerSecond > _swerveParser._theoreticalMaxWheelSpeedMPS)
        return true;
    }
    return false;
  }

  public void fieldRelativeDrive(
      double xTranslationalMPS, double yTranslationalMPS, double rotationRadPS
  ) {
    fieldRelativeDrive(
        xTranslationalMPS,
        yTranslationalMPS,
        rotationRadPS,
        _swerveParser._theoreticalMaxWheelSpeedMPS
    );
  }

  public void fieldRelativeDrive(
      double xTranslationalMPS, double yTranslationalMPS, double rotationRadPS, double maxSpeedMPS
  ) {
    robotCentricDrive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xTranslationalMPS,
            yTranslationalMPS,
            rotationRadPS,
            USE_POSE_ESTIMATION_ANGLE ? getPose().getRotation() : getYaw()
        ),
        maxSpeedMPS
    );
  }

  public void fieldRelativeAntiscrubDrive(
      double xTranslationalMPS, double yTranslationalMPS, double rotationRadPS
  ) {
    fieldRelativeAntiscrubDrive(
        xTranslationalMPS, yTranslationalMPS, rotationRadPS, Drivetrain.ALLOWED_SCRUB
    );
  }

  public void fieldRelativeAntiscrubDrive(
      double xTranslationalMPS, double yTranslationalMPS, double rotationRadPS, double allowedScrub
  ) {
    fieldRelativeAntiscrubDrive(
        xTranslationalMPS,
        yTranslationalMPS,
        rotationRadPS,
        allowedScrub,
        _swerveParser._theoreticalMaxWheelSpeedMPS
    );
  }

  public void fieldRelativeAntiscrubDrive(
      double xTranslationalMPS,
      double yTranslationalMPS,
      double rotationRadPS,
      double allowedScrub,
      double maxSpeedMPS
  ) {
    robotRelativeAntiscrubDrive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xTranslationalMPS,
            yTranslationalMPS,
            rotationRadPS,
            USE_POSE_ESTIMATION_ANGLE ? getPose().getRotation() : getYaw()
        ),
        allowedScrub,
        maxSpeedMPS
    );
  }

  public void robotRelativeAntiscrubDrive(
      ChassisSpeeds desiredChassisSpeeds, double allowedScrub, double maxSpeedMPS
  ) {
    if (_isSubsystemDisabled) {
      return;
    }
    _previousModuleSetpoint = _setpointGenerator.getFeasibleSetpoint(
        _previousModuleSetpoint, correctForDynamics(desiredChassisSpeeds), allowedScrub
    );
    setModuleStates(_previousModuleSetpoint.moduleStates, maxSpeedMPS);
  }

  private void rotationCompensationFieldRelativeDrive(
      double percentX, double percentY, double percentR, double maxSpeedMPS
  ) {
    // args
    double configuredMaxTranslationalSpeed = 0;
    double configuredMaxRotationalSpeed = 0;

    // consts
    // these need to be robot relative if guaranteeing the angle
    ChassisSpeeds robotRelative =
        ChassisSpeeds.fromFieldRelativeSpeeds(percentX, percentY, percentR, getYaw());
    // robotRelative.
    double scaledX = robotRelative.vxMetersPerSecond * configuredMaxTranslationalSpeed;
    double scaledY = robotRelative.vyMetersPerSecond * configuredMaxTranslationalSpeed;
    double percentXY = Math.hypot(percentX, percentY);
    double scaledXY = percentXY * configuredMaxTranslationalSpeed;
    double absScaledX = Math.abs(scaledX);
    double absScaledY = Math.abs(scaledY);
    double scaledR = robotRelative.omegaRadiansPerSecond * configuredMaxRotationalSpeed;
    double rWheel =
        scaledR * configuredMaxTranslationalSpeed / _swerveParser._theoreticalMaxRotationalSpeedDPS;
    double projected =
        (Math.min(absScaledX, absScaledY) * 2 + Math.abs(absScaledX - absScaledY)) / Math.sqrt(2.0);

    // We should rescale
    if (projected + rWheel > _swerveParser._theoreticalMaxTranslationSpeedMPS) {
      // // projected oversaturation values (using projected vector value)
      // // no because this is dependent on the direction of translation
      // // which will change with constant input
      // double maxOverSaturation =
      // (-1 + (configuredMaxRotationalSpeed /
      // DRIVE_CONSTS.theoreticalMaxRotationalSpeed)
      // + (configuredMaxTranslationalSpeed /
      // DRIVE_CONSTS.theoreticalMaxTranslationSpeed));
      // double overSaturation =
      // -1 + (projected + rWheel) / DRIVE_CONSTS.theoreticalMaxTranslationSpeed;
      // double scalingCoefficient = overSaturation / maxOverSaturation;

      // unprojected value
      double scalingCoefficient = -1 + (scaledXY / _swerveParser._theoreticalMaxTranslationSpeedMPS)
          + (scaledR / _swerveParser._theoreticalMaxRotationalSpeedDPS);
      double newScaledXY = scaledXY
          + (_swerveParser._theoreticalMaxTranslationSpeedMPS - configuredMaxTranslationalSpeed)
              * percentXY * scalingCoefficient;
      double newScaledR = scaledR
          + (_swerveParser._theoreticalMaxRotationalSpeedDPS - configuredMaxRotationalSpeed);
      scaledX = scaledX * newScaledXY / scaledXY;
      scaledY = scaledY * newScaledXY / scaledXY;
      scaledR = newScaledR;
    }

    fieldRelativeDrive(scaledX, scaledY, scaledR, maxSpeedMPS);
  }

  private void slewingFieldRelativeDrive(
      double xTranslationalMPS, double yTranslationalMPS, double rotationRadPS, double maxSpeedMPS
  ) {
    fieldRelativeDrive(
        _xSlewRateFilter.calculate(xTranslationalMPS),
        _ySlewRateFilter.calculate(yTranslationalMPS),
        _angleSlewRateFilter.calculate(rotationRadPS),
        maxSpeedMPS
    );
  }

  private void snapToAngleDrive(
      double xTranslationalMPS, double yTranslationalMPS, double maxSpeedMPS, double maxAngularSpeed
  ) {
    if (_isSubsystemDisabled) {
      return;
    }
    var theta = AlgebraicUtils.clamp(
        _angleController.calculate(getYaw().getRadians()), -maxAngularSpeed, maxAngularSpeed
    );
    fieldRelativeDrive(
        xTranslationalMPS,
        yTranslationalMPS,
        // calculate snap to angle rotation here
        // TODO: reconfigure deadband logic to limit jittering
        _angleController.atSetpoint() ? 0.0 : theta,
        maxSpeedMPS
    );
  }

  public void snapToAngleDrive(double xTranslationalMPS, double yTranslationalMPS) {
    snapToAngleDrive(
        xTranslationalMPS, yTranslationalMPS, _translationalLimitsM.maxVelocity, _angularMaxVeloRad
    );
  }

  private void snakeDrive(
      double controllerX,
      double controllerY,
      double compositeXY,
      double maxSpeedMPS,
      double maxAngularSpeed
  ) {
    if (compositeXY > Control.STICK_NET_DEADBAND) {
      _angleController.reset(
          getPose().getRotation().getRadians(), _chassisSpeeds.omegaRadiansPerSecond
      );
      if (controllerX == 0.0) {
        _angleController.setGoal(
            controllerY > 0.0 ? Units.degreesToRadians(90.0) : Units.degreesToRadians(270.0)
        );
      } else {
        _angleController.setGoal(Math.atan2(controllerY, controllerX));
      }
    }

    snapToAngleDrive(
        InputUtils.scaleJoystickXMPS(controllerX, compositeXY, maxSpeedMPS),
        InputUtils.scaleJoystickYMPS(controllerY, compositeXY, maxSpeedMPS),
        maxSpeedMPS,
        maxAngularSpeed
    );
  }

  private void snakeDrive(double controllerX, double controllerY) {
    snakeDrive(controllerX, controllerY, Math.hypot(controllerX, controllerY));
  }

  private void snakeDrive(double xTranslationalMPS, double yTranslationalMPS, double compositeXY) {
    snakeDrive(
        xTranslationalMPS,
        yTranslationalMPS,
        compositeXY,
        _translationalLimitsM.maxVelocity,
        _angularMaxVeloRad
    );
  }

  private void fieldRelativeDriveCompensation(
      double xTranslationalMPS, double yTranslationalMPS, double rotationRadPS, double maxSpeedMPS
  ) {
    double x_meter = xTranslationalMPS * Constants.LOOP_PERIOD_S;
    double y_meter = yTranslationalMPS * Constants.LOOP_PERIOD_S;
    double theta_deg = rotationRadPS * Constants.LOOP_PERIOD_S;
    Pose2d goal = new Pose2d(x_meter, y_meter, Rotation2d.fromDegrees(theta_deg));
    Twist2d twistTranslation = new Pose2d().log(goal);
    fieldRelativeDrive(
        twistTranslation.dx / Constants.LOOP_PERIOD_S,
        twistTranslation.dy / Constants.LOOP_PERIOD_S,
        twistTranslation.dtheta / Constants.LOOP_PERIOD_S,
        maxSpeedMPS
    );
  }

  private void fieldRelativeDriveCompensation(
      double xTranslationalMPS, double yTranslationalMPS, double rotationRadPS
  ) {
    fieldRelativeDriveCompensation(
        xTranslationalMPS, yTranslationalMPS, rotationRadPS, _translationalLimitsM.maxVelocity
    );
  }

  public void setTolerance(double xTolerance, double yTolerance, Rotation2d thetaTolerance) {
    _xController.setTolerance(xTolerance);
    _yController.setTolerance(yTolerance);
    _angleController.setTolerance(thetaTolerance.getRadians());
  }

  public void resetDefaultTolerance() {
    setTolerance(_translationalToleranceM, _translationalToleranceM, _rotationalToleranceRot);
  }

  /*
   * @param translationX, translationY apply to entire robot
   *
   * @param rotation measured from center of rotation
   *
   * @param target point of rotation
   *
   * @param maxSpeed
   */
  private void targetCentricDrive(
      double translationX,
      double translationY,
      double rotation,
      Translation2d target,
      double maxSpeed
  ) {
    // TODO: THIS IS JUST WRONG
    SwerveModuleState[] goalModuleStates = _swerveKinematics.toSwerveModuleStates(
        new ChassisSpeeds(translationX, translationY, rotation), target
    );

    setModuleStates(goalModuleStates, maxSpeed);
  }

  public void chaseStaticTargetDrive(double maxSpeedMPS) {
    if (_isSubsystemDisabled) {
      return;
    }
    Pose2d currentPose = getPose();

    if (RobotAltModes.isUnprofiledPIDMode) {
      // TODO Test/replace with slewing drive (?)
      fieldRelativeDrive(
          _xController.atSetpoint() ? 0.0 : _xController.calculate(currentPose.getX()),
          _yController.atSetpoint() ? 0.0 : _yController.calculate(currentPose.getY()),
          _angleController.atSetpoint() ? 0.0 : _angleController.calculate(getYaw().getRadians()),
          maxSpeedMPS
      );
    } else {
      // Profiled controllers naturally filter/slew
      if (RobotAltModes.isPoseTuning) {
        _xPoseError.setDouble(_xController.getPositionError());
        _yPoseError.setDouble(_yController.getPositionError());
        _thetaPoseError.setDouble(Units.radiansToDegrees(_angleController.getPositionError()));
      }

      fieldRelativeDrive(
          _xController.calculate(currentPose.getX()),
          _yController.calculate(currentPose.getY()),
          _angleController.calculate(getYaw().getRadians()),
          maxSpeedMPS
      );
    }
  }

  public void chaseStaticTargetDrive() {
    chaseStaticTargetDrive(_translationalLimitsM.maxVelocity);
  }

  private void forceWheelChaseStaticTargetDrive(double maxSpeedMPS) {
    Pose2d currentPose = getPose();

    if (RobotAltModes.isUnprofiledPIDMode) {
      // TODO Test/replace with slewing drive (?)
      forceWheelDirection(
          _xController.atSetpoint() ? 0 : _xController.calculate(currentPose.getX()),
          _yController.atSetpoint() ? 0 : _yController.calculate(currentPose.getY()),
          _angleController.atSetpoint() ? 0 : _angleController.calculate(getYaw().getRadians()),
          maxSpeedMPS
      );
    } else {
      // Profiled controllers naturally filter/slew
      forceWheelDirection(
          _xController.calculate(currentPose.getX()),
          _yController.calculate(currentPose.getY()),
          _angleController.calculate(getYaw().getRadians()),
          maxSpeedMPS
      );
    }
  }

  private void forceWheelChaseStaticTargetDrive() {
    forceWheelChaseStaticTargetDrive(_translationalLimitsM.maxVelocity);
  }

  private void chaseDynamicTargetDrive(
      Pose2d visionTarget, Pose2d relativeGoal, boolean targetMoved, double maxSpeedMPS
  ) {
    if (targetMoved) {
      Transform2d goalPose = visionTarget.minus(relativeGoal);

      if (RobotAltModes.isUnprofiledPIDMode) {
        _xController.reset(goalPose.getX());
        _yController.reset(goalPose.getY());
        _angleController.reset(goalPose.getRotation().getRadians());
      } else {
        Pose2d currentPose = getPose();
        _xController.reset(currentPose.getX(), _chassisSpeeds.vxMetersPerSecond);
        _yController.reset(currentPose.getY(), _chassisSpeeds.vyMetersPerSecond);
        _angleController.reset(
            currentPose.getRotation().getRadians(), _chassisSpeeds.omegaRadiansPerSecond
        );
        _xController.setGoal(goalPose.getX());
        _yController.setGoal(goalPose.getY());
        _angleController.setGoal(goalPose.getRotation().getRadians());
      }
    }
    chaseStaticTargetDrive(maxSpeedMPS);
  }

  private void chaseDynamicTargetDrive(
      Pose2d visionTarget, Pose2d relativeGoal, boolean targetMoved
  ) {
    chaseDynamicTargetDrive(
        visionTarget, relativeGoal, targetMoved, _swerveParser._theoreticalMaxWheelSpeedMPS
    );
  }

  private void forceWheelDirection(
      double xTranslationalMPS, double yTranslationalMPS, double rotationRadPS, double maxSpeedMPS
  ) {
    SwerveModuleState[] goalModuleStates =
        _swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
            xTranslationalMPS, yTranslationalMPS, rotationRadPS, getYaw()
        ));

    for (int i = 0; i < _swerveParser._numModules; i++)
      _modules[i].setLastAngle(goalModuleStates[i].angle);

    if (areWheelsAligned(goalModuleStates)) {
      // Desaturate based of max theoretical or functional rather than current max
      edu.wpi.first.math.kinematics.SwerveDriveKinematics.desaturateWheelSpeeds(
          goalModuleStates, _swerveParser._theoreticalMaxWheelSpeedMPS
      );

      for (var module : _modules) {
        module.setDesiredState(
            goalModuleStates[module._moduleNumber], Constants.Control.IS_OPEN_LOOP
        );
      }

      // _desiredSpeed->SetDouble(goalModuleStates[0].speed());
      setDesiredSwerveState(goalModuleStates);
    } else {
      fieldRelativeDrive(0.0, 0.0, 0.0, maxSpeedMPS);
      // _desiredSpeed->SetDouble(0.0);
    }
  }

  private void forceWheelDirection(
      double xTranslationalMPS, double yTranslationalMPS, double rotationRadPS
  ) {
    forceWheelDirection(
        xTranslationalMPS,
        yTranslationalMPS,
        rotationRadPS,
        _swerveParser._theoreticalMaxWheelSpeedMPS
    );
  }

  private void forceWheelDirectionDrive(
      double moduleAngleDeg, double speedMPS, double maxSpeedMPS
  ) {
    speedMPS = Math.min(speedMPS, maxSpeedMPS);
    Rotation2d moduleAngle = Rotation2d.fromDegrees(moduleAngleDeg);
    SwerveModuleState goalModuleState = new SwerveModuleState(speedMPS, moduleAngle);

    for (var module : _modules) module.setLastAngle(moduleAngle);

    if (areWheelsAligned(goalModuleState)) {
      for (var module : _modules)
        module.setDesiredState(goalModuleState, Constants.Control.IS_OPEN_LOOP);

      // _desiredSpeed->SetDouble(goalModuleState.speed());
    } else {
      fieldRelativeDrive(0.0, 0.0, 0.0, maxSpeedMPS);
      // _desiredSpeed->SetDouble(0.0);
    }
  }

  private void forceWheelDirectionDrive(double moduleAngleDeg, double speedMPS) {
    forceWheelDirectionDrive(moduleAngleDeg, speedMPS, _translationalLimitsM.maxVelocity);
  }

  public void setSnapAngleDeg(double angleDeg) {
    setSnapAngleRad(Units.degreesToRadians(angleDeg));
  }

  public void setSnapAngle(Rotation2d angle) {
    setSnapAngleRad(angle.getRadians());
  }

  public void setSnapAngleRad(double angleRad) {
    _angleController.reset(
        getPose().getRotation().getRadians(), _chassisSpeeds.omegaRadiansPerSecond
    );
    _angleController.setGoal(angleRad);
  }

  // TODO: replace with real face target cmd
  public Command snapAngleCommand(Rotation2d angle) {
    return new InstantCommand(() -> {
      setSnapAngle(angle);
      snapToAngleDrive(
          _chassisSpeeds.vxMetersPerSecond,
          _chassisSpeeds.vyMetersPerSecond,
          _translationalLimitsM.maxVelocity,
          _angularMaxVeloRad
      );
    });
  }

  public void setStaticTarget(Pose2d goalPose) {
    Pose2d currentPose = getPose();
    _xController.reset(currentPose.getX(), _chassisSpeeds.vxMetersPerSecond);
    _yController.reset(currentPose.getY(), _chassisSpeeds.vyMetersPerSecond);
    _angleController.reset(
        currentPose.getRotation().getRadians(), _chassisSpeeds.omegaRadiansPerSecond
    );
    _xController.setGoal(goalPose.getX());
    _yController.setGoal(goalPose.getY());
    _angleController.setGoal(goalPose.getRotation().getRadians());
  }

  public Rotation2d toAbsoluteAngle(Rotation2d angle) {
    return toAbsoluteAngle(angle);
  }

  public double toAbsoluteAngleRad(double angleRad) {
    return Units.degreesToRadians(toAbsoluteAngleDeg(Units.radiansToDegrees(angleRad)));
  }

  public double toAbsoluteAngleDeg(double angleDeg) {
    double scaled = (angleDeg % 360.0);
    return scaled + (scaled < 0 ? 360.0 : 0.0);
  }

  public void testSteer(double angleRot) {
    for (var module : _modules) module.setLastAngleRot(angleRot);
  }

  public void forceScoreWheelDirection() {
    testSteer(0.0);
  }

  public void forceScoreWheelDirection(double angleDeg) {}

  public void forceChargingWheelDirection() {
    var goalModuleStates = _swerveKinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 90, getYaw())
    );
    for (int i = 0; i < _swerveParser._numModules; i++) {
      goalModuleStates[i].angle = goalModuleStates[i].angle.plus(Rotation2d.fromDegrees(90.0));
      _modules[i].setLastAngle(goalModuleStates[i]);
    }
  }

  public boolean inRange() {
    return (_xController.atGoal() && _yController.atGoal() && _angleController.atGoal());
  }

  public boolean thetaInRange() {
    return _angleController.atGoal();
  }

  public void updateTuneScoringShuffleboard(double xError, double yError, double thetaError) {
    if (RobotAltModes.isPoseTuning) {
      _xPoseError.setDouble(xError);
      _yPoseError.setDouble(yError);
      _thetaPoseError.setDouble(thetaError);
    }
  }

  public void updateTuneScoringStatus(boolean tuneScoring) {
    if (RobotAltModes.isPoseTuning) {
      _usingPoseTuning.setBoolean(tuneScoring);
    }
  }

  public InstantCommand zeroGyroCommand() {
    return new InstantCommand(() -> zeroGyro());
  }

  public InstantCommand zeroPoseCommand() {
    return new InstantCommand(() -> zeroPose());
  }

  public InstantCommand forceChargingWheelDirectionCommand() {
    return new InstantCommand(() -> forceChargingWheelDirection());
  }

  @Override
  public boolean checkInitStatus() {
    StatusCode initStatus = _gyro.getConfigurator().apply(_gyroConfigs);
    return (initStatus == StatusCode.OK);
  }

  private void resetSimPose(Pose2d pose) {
    _simPose = pose;
  }

  // // Gets vision pose of the primary camera
  public Pose2d getVisionPose() {
    return _multiCameraController.getVisionPose();
  }

  public double getVisionCenterPixelController(double _centerOffsetX) {
    return _visionCenterOffsetController.calculate(_centerOffsetX);
  }

  public double getCenterOffsetX(VisionModes visionMode) {
    return _multiCameraController.getCenterOffsetX(visionMode, Controlboard.isRedAlliance());
  }

  public VisionModes getVisionMode() {
    return _multiCameraController.getVisionMode();
  }

  public Pose2d getPiecePose() {
    return _multiCameraController.getPiecePose(getPose());
  }

  public void setVisionMode(VisionModes visionMode) {
    _multiCameraController.setVisionMode(visionMode);
  }

  public void setPrematchVisionMode() {
    _multiCameraController.setVisionMode(VisionModes.PREMATCH);
  }

  public void setAutoVisionMode() {
    _multiCameraController.setVisionMode(VisionModes.AUTO);
  }

  public void setTeleopVisionMode() {
    _multiCameraController.setVisionMode(VisionModes.TELEOP);
  }

  public void setShootingVisionMode() {
    _multiCameraController.setVisionMode(VisionModes.SHOOTING);
  }

  public void setAmpVisionMode() {
    _multiCameraController.setVisionMode(VisionModes.AMP);
  }

  public void setClimbVisionMode() {
    _multiCameraController.setVisionMode(VisionModes.CLIMB);
  }

  public void setHPVisionMode() {
    _multiCameraController.setVisionMode(VisionModes.HP_INTAKE);
  }

  public void setGroundVisionMode() {
    _multiCameraController.setVisionMode(VisionModes.GROUND_INTAKE);
  }

  public InstantCommand forceSetInitalPoseCommand() {
    return new InstantCommand(() -> {
      setPose(new Pose2d(
          new Pose2d().getTranslation().plus(new Translation2d(-1.0, 0.0)),
          Rotation2d.fromDegrees(180.0)
      ));
    });
  }

  public Command forceAllianceBasedFieldRelativeMovementCommand(
      double blueXMPS, double blueYMPS, double timeout_s
  ) {
    return forceFieldRelativeMovementCommand(
        blueXMPS,
        blueYMPS,
        -blueXMPS,
        -blueYMPS,
        timeout_s,
        _swerveParser._theoreticalMaxWheelSpeedMPS
    );
  }

  public Command forceFieldRelativeMovementCommand(double xMPS, double yMPS, double timeout_s) {
    return forceFieldRelativeMovementCommand(
        xMPS, yMPS, xMPS, yMPS, timeout_s, _swerveParser._theoreticalMaxWheelSpeedMPS
    );
  }

  public Command forceFieldRelativeMovementCommand(
      double blueXMPS,
      double blueYMPS,
      double redXMPS,
      double redYMPS,
      double timeout_s,
      double maxSpeedMPS
  ) {
    // run rather than Commands.run so this implicity requires the current subsystem
    // (drivertrain)
    return run(() -> {
             fieldRelativeDrive(
                 Controlboard.isRedAlliance() ? redXMPS : blueXMPS,
                 Controlboard.isRedAlliance() ? redYMPS : blueYMPS,
                 0.0,
                 maxSpeedMPS
             );
           }
    ).withTimeout(timeout_s);
  }

  public Command forceRobotRelativeMovementCommand(double xMPS, double yMPS, double timeout_s) {
    return forceRobotRelativeMovementCommand(
        xMPS, yMPS, timeout_s, _swerveParser._theoreticalMaxWheelSpeedMPS
    );
  }

  public Command forceRobotRelativeMovementCommand(
      double XMPS, double YMPS, double timeout_s, double maxSpeedMPS
  ) {
    // run rather than Commands.run so this implicity requires the current subsystem
    // (drivertrain)
    return run(() -> { robotCentricDrive(XMPS, YMPS, 0.0, maxSpeedMPS); }).withTimeout(timeout_s);
  }

  // Threaded odometry
  public class OdometryThread extends Thread {
    private static int kThreadPriority = 1;
    private BaseStatusSignal[] _allSignals;
    public int _successfulDaqs = 0;
    public int _failedDaqs = 0;

    // Track thread timing
    private final MedianFilter _peakRemover = new MedianFilter(3);
    private final LinearFilter _lowPass = LinearFilter.movingAverage(50);
    private double _lastTime = 0.0;

    public OdometryThread(final TagalongSwerveModuleBase[] modules) {
      super();
      setDaemon(true);

      // 4 signals for each module + 2 for Pigeon2.
      _allSignals = new StatusSignal[(modules.length * 4) + 2];
      for (int i = 0; i < modules.length; i++) {
        BaseStatusSignal[] signals = modules[i].getSignals();
        int baseIndex = i * 4;
        for (int j = 0; j < 4; j++) _allSignals[baseIndex + j] = signals[j];
      }

      _allSignals[_allSignals.length - 2] = _gyro.getYaw().clone();
      _allSignals[_allSignals.length - 1] = _gyro.getAngularVelocityZWorld().clone();

      /* Make sure all signals update at around 250hz */
      while (
          BaseStatusSignal.setUpdateFrequencyForAll(RobotAltModes.kOdometryFrequency, _allSignals)
          != StatusCode.OK
      ) {
        System.out.println("setUpdateFrequencyForAll not ok");
      }
      Threads.setCurrentThreadPriority(true, 1);
    }

    @Override
    public void run() {
      Threads.setCurrentThreadPriority(true, kThreadPriority);

      // Run as fast as possible, the blocking for the signals will control the
      // timing.
      StatusCode status;
      while (true) {
        if (RobotAltModes.kEnableLatencyCompensation) {
          // Synchronously wait for all signals, up to twice the period of the update
          // frequency.
          status = Robot.isReal()
              ? StatusSignal.waitForAll(2.0 / RobotAltModes.kOdometryFrequency, _allSignals)
              : StatusCode.OK;
        }

        if (status != StatusCode.OK) {
          if (Robot.isReal()) {
            System.err.println("!!");
          }
        } else {
          _odometryLock.writeLock().lock();

          // Compute loop stats
          final double currentTime = Timer.getFPGATimestamp();
          _averageOdometryLoopTime =
              _lowPass.calculate(_peakRemover.calculate(currentTime - _lastTime));
          _lastTime = currentTime;

          /* Get status of first element */
          if (status.isOK()) {
            _successfulDaqs++;
          } else {
            _failedDaqs++;
          }

          /* Now update odometry */
          /* Keep track of the change in azimuth rotations */
          for (int i = 0; i < _modules.length; i++) {
            _modulePositions[i] = _modules[i].getPosition(false);
          }
          // Assume Pigeon2 is flat-and-level so latency compensation can be performed
          // TODO FIX THREAD SAFETY
          _cachedState._yaw = Rotation2d.fromDegrees(BaseStatusSignal.getLatencyCompensatedValue(
              _gyro.getYaw(), _gyro.getAngularVelocityZWorld()
          ));

          /* Keep track of previous and current pose to account for the carpet vector */
          _robotPoseEstimator.updateWithTime(currentTime, _cachedState._yaw, _modulePositions);
          _visionSwervePoseEstimator.updateWithTime(
              currentTime, _cachedState._yaw, _modulePositions
          );

          // TODO
          // /* And now that we've got the new odometry, update the controls */
          // _requestParameters.currentPose =
          // _robotPoseEstimator.getEstimatedPosition().relativeTo(
          // new Pose2d(0, 0, _fieldRelativeOffset)
          // );
          // _requestParameters.kinematics = _swerveKinematics;
          // _requestParameters.swervePositions = _swerveModuleLocations;
          // _requestParameters.timestamp = currentTime;
          // _requestParameters.updatePeriod = 1.0 / RobotAltModes.kOdometryFrequency;

          // setModuleStates(_requestParameters)
          // _requestToApply.apply(_requestParameters, _modules);

          /* Update our cached state with the newly updated data */
          _cachedState._failedDaqs = _failedDaqs;
          _cachedState._successfulDaqs = _successfulDaqs;
          _cachedState._moduleStates = new SwerveModuleState[_modules.length];
          for (int i = 0; i < _modules.length; ++i) {
            _cachedState._moduleStates[i] = _modules[i].getState();
          }
          _cachedState._estimatedPose = _robotPoseEstimator.getEstimatedPosition();

          // TODO: call an update function and pre-construct the array
          _chassisSpeeds = _swerveKinematics.toChassisSpeeds(_cachedState._moduleStates);

          _cachedState._odometryPeriod = _averageOdometryLoopTime;

          if (_telemetryFunction != null) {
            /* Log our state */
            _telemetryFunction.accept(_cachedState);
          }

          // TODO:604 Output stats
          // // Only update localizer when enabled
          // if (DriverStation.isEnabled()) {
          // final double yaw = Constants.kEnableLatencyCompensation
          // ? StatusSignal.getLatencyCompensatedValue(
          // _gyro.continuousYawSignal(), _gyro.yawRateSignal()
          // )
          // : _gyro.getContinuousYaw();
          // final var odometryMeasurement =
          // new Pose2d(new Rotation2d(yaw), getModulePositionStates(false));
          // m_localizer.update(odometryMeasurement, new ArrayList<>());
          // }

          // // Apply closed-loop controls
          // // TODO: this is a temp hack for testing
          // if (Constants.kEnableSynchronousOutput &&
          // DriverStation.isAutonomousEnabled()) {
          // final ChassisSpeeds fieldRelativeChassisSpeeds = _driveController.calculate(
          // RobotAltModes.getPose(), m_targetPose, m_xVelRef, m_yVelRef, m_thetaVelRef
          // );
          // driveOpenLoop(
          // fieldRelativeChassisSpeeds.vxMetersPerSecond,
          // fieldRelativeChassisSpeeds.vyMetersPerSecond,
          // fieldRelativeChassisSpeeds.omegaRadiansPerSecond,
          // /*fieldRelative=*/false,
          // m_allowedScrub,
          // 1.0 / RobotAltModes.kOdometryFrequency
          // );
          // }

          _odometryLock.writeLock().unlock();
        }
      }
    }

    public boolean odometryIsValid() {
      return _successfulDaqs > 2; // Wait at least 3 daqs before saying the odometry is valid
    }

    /**
     * Sets the DAQ thread priority to a real time priority under the specified
     * priority level
     *
     * @param priority Priority level to set the DAQ thread to.
     *                 This is a value between 0 and 99, with 99 indicating higher
     *                 priority and
     *                 0 indicating lower priority.
     */
    public void setThreadPriority(int priority) {
      kThreadPriority = priority;
    }
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    try {
      _odometryLock.readLock().lock();
      return _chassisSpeeds;
    } finally {
      _odometryLock.readLock().unlock();
    }
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), getYaw());
  }

  public double getFaceTargetRad() {
    return getFaceTargetRad(_faceTargetGoal);
  }

  public double getFaceTargetRad(Translation2d target) {
    return getFaceTargetRad(target, getPose().getTranslation());
  }

  public static double getFaceTargetRad(Translation2d target, Translation2d currLocation) {
    return getDesiredYawRad(target.getX(), target.getY(), currLocation.getX(), currLocation.getY());
  }

  public static double ffChassisMovement(
      double targetX, double targetY, double robotX, double robotY
  ) {
    return getDesiredYawRad(targetX, targetY, robotX, robotY);
  }

  public static double getDesiredYawRad(
      double targetX, double targetY, double robotX, double robotY
  ) {
    double deltaX = targetX - robotX;
    double deltaY = targetY - robotY;

    if (deltaY == 0) {
      return deltaX >= 0 ? 0.0 : Math.PI;
    } else {
      return Math.atan2(deltaY, deltaX);
    }
  }

  public double updateFaceAngleTargetRadAndGetPID(double currentYaw) {
    _angleController.setGoal(getFaceTargetRad());
    return _angleController.calculate(currentYaw);
  }

  public void faceTargetMode(double xTranslationalMPS, double yTranslationalMPS) {
    faceTargetMode(
        xTranslationalMPS, yTranslationalMPS, _translationalLimitsM.maxVelocity, _angularMaxVeloRad
    );
  }

  public void faceTargetMode(
      double xTranslationalMPS, double yTranslationalMPS, double maxSpeed, double maxAngularSpeed
  ) {
    _angleController.setGoal(getFaceTargetRad());

    snapToAngleDrive(xTranslationalMPS, yTranslationalMPS, maxSpeed, maxAngularSpeed);
  }

  public void setFaceTarget(Translation2d target) {
    _faceTargetGoal = target;

    _angleController.reset(
        getPose().getRotation().getRadians(), _chassisSpeeds.omegaRadiansPerSecond
    );
  }

  public Translation2d velocityCompensation2d(
      Translation2d robot,
      ChassisSpeeds fieldRelativeSpeeds,
      ChassisSpeeds desiredFieldRelativeSpeeds,
      double deltaS
  ) {
    var averageSpeeds = fieldRelativeSpeeds.plus(desiredFieldRelativeSpeeds).div(2.0);
    return new Translation2d(averageSpeeds.vxMetersPerSecond, averageSpeeds.vyMetersPerSecond)
        .times(deltaS)
        .plus(robot);
  }

  public void disabledPeriodic() {
    if (!_isSubsystemDisabled) {
      for (var module : _modules) {
        module.disabledPeriodic();
      }
    }
  }

  @Override
  public void onEnable() {
    for (var module : _modules) {
      module.onEnable();
    }
  }

  @Override
  public void onDisable() {
    // _multiCameraController._currentVisionState = VisionState.SINGLE;
    for (var module : _modules) {
      module.onDisable();
    }
  }

  public boolean atPose(Pose2d currentPose, Pose2d targetPose) {
    var dif = currentPose.relativeTo(targetPose);
    return dif.getX() <= _xController.getPositionTolerance()
        && dif.getY() <= _yController.getPositionTolerance()
        && dif.getRotation().getDegrees() <= _angleController.getPositionTolerance();
  }

  public Pigeon2 getGyro() {
    return _gyro;
  }

  public int getMaxTargetsSeen() {
    return _maxTargetsSeen;
  }
}
