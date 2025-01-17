package frc.robot.commands.complex;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotAltModes;
import frc.robot.constants.enums.IntakePivotPositions;
import frc.robot.constants.enums.ScoreTargets;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.InputUtils;
import tagalong.math.AlgebraicUtils;
import tagalong.math.GeometricUtils;
import tagalong.measurements.AlliancePose3d;

public class SOTFCommand extends Command {
  /* circ of small wheel * reduction */
  public static final double NOTE_INIT_MPS_COEFFICIENT = 9.0;

  // Units.inchesToMeters(35.0 * 2.875 * Math.PI * 66.0 / 50.0);
  // unclear what to use -- threshold or actual velocity
  public static final double MAX_TARGET_RANGE_M = 6.5;
  public static final double MIN_ANGULAR_RANGE_ROT = 0.6 / 360.0;
  public static final double MAX_SPEED_MPS = 2.5;
  protected final Drivetrain _drivetrain;
  protected final Shooter _shooter;
  protected final Intake _intake;
  public static final Controlboard _controlboard = Controlboard.get();
  protected final AlliancePose3d _middleAllianceTargets, _leftAllianceTargets,
      _rightAllianceTargets;

  protected Translation2d _target2d, _leftTarget2d, _rightTarget2d;
  protected Translation2d _veloCompTarget, _leftVeloCompTarget, _rightVeloCompTarget;
  protected ChassisSpeeds _frSpeeds;
  protected Pose2d _currentPose;
  protected Translation2d _currentTranslation;
  protected double _prevDesiredYawRad;
  protected PIDController _angleController;
  protected Rotation2d _currentYaw;
  public static boolean _onYawTarget, _onPivotTarget, _inTargetRange;
  protected TrapezoidProfile _rotationProfile;
  protected double _prevNetRotationalRadPS;
  protected TrapezoidProfile.State _targetState = new TrapezoidProfile.State();
  private LinearFilter _filter = LinearFilter.movingAverage(50);
  private double _TTO;
  private Translation2d _frTranslation;
  private double _currentNoteSpeedHorizontal, _currentNoteSpeedVertical;
  private double _xVelocityMPS, _yVelocityMPS;
  private Timer _timer = new Timer();
  public static double _lastDistance;

  public SOTFCommand(
      Drivetrain drivetrain,
      Shooter shooter,
      Intake intake,
      ScoreTargets targets,
      boolean velocityCompensation,
      ScoreTargets leftTarget,
      ScoreTargets rightTarget
  ) {
    _drivetrain = drivetrain;
    _shooter = shooter;
    _intake = intake;
    _middleAllianceTargets = targets.target;
    _leftAllianceTargets = leftTarget.target;
    _rightAllianceTargets = rightTarget.target;

    addRequirements(drivetrain);
    _angleController = new PIDController(0.8, 0.0, 0.0);
    _angleController.enableContinuousInput(0.0, 2 * Math.PI);
  }

  public SOTFCommand(Drivetrain drivetrain, Shooter shooter, Intake intake) {
    this(
        drivetrain,
        shooter,
        intake,
        ScoreTargets.SPEAKER_BOTTOM,
        true,
        ScoreTargets.SPEAKER_LEFT_BOTTOM,
        ScoreTargets.SPEAKER_RIGHT_BOTTOM
    );
  }

  @Override
  public void initialize() {
    _target2d = _middleAllianceTargets.getTranslation2d(_controlboard.isRedAlliance());
    _leftTarget2d = _leftAllianceTargets.getTranslation2d(_controlboard.isRedAlliance());
    _rightTarget2d = _rightAllianceTargets.getTranslation2d(_controlboard.isRedAlliance());
    _onYawTarget = false;
    _onPivotTarget = false;

    updatePositionsAndCompensatedTargets();

    _prevDesiredYawRad = AlgebraicUtils.placeInScopeRad(
        _drivetrain.getYaw().getRadians(),
        Drivetrain.getDesiredYawRad(
            _veloCompTarget.getX(), _veloCompTarget.getY(), _currentPose.getX(), _currentPose.getY()
        )
    );
    _angleController.setSetpoint(_prevDesiredYawRad);

    _rotationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
        _drivetrain._angularMaxVeloRad, _drivetrain._angularMaxAccelRad
    ));

    _targetState.position = _drivetrain.getYaw().getRadians();
    _targetState.velocity = 0.0;
  }

  private void updatePositionsAndCompensatedTargets() {
    _currentPose = _drivetrain.getPose();
    _currentTranslation = _currentPose.getTranslation();
    _currentYaw = _drivetrain.getYaw();

    double currentTargetAngleRad = _shooter.getTargetLaunchAngleFromTargetLinearizedRot(
        _currentTranslation.getDistance(_target2d)
    );
    _currentNoteSpeedHorizontal = NOTE_INIT_MPS_COEFFICIENT * Math.cos(currentTargetAngleRad);
    _currentNoteSpeedVertical = NOTE_INIT_MPS_COEFFICIENT * Math.sin(currentTargetAngleRad);

    _frSpeeds = _drivetrain.getFieldRelativeSpeeds();
    // TODO make dynamic
    _TTO = _currentTranslation.getDistance(_target2d) / _currentNoteSpeedHorizontal;
    _frTranslation =
        new Translation2d(_frSpeeds.vxMetersPerSecond * _TTO, _frSpeeds.vyMetersPerSecond * _TTO);
    _veloCompTarget = _target2d.minus(_frTranslation);

    _leftVeloCompTarget = _leftTarget2d.minus(_frTranslation);
    _rightVeloCompTarget = _rightTarget2d.minus(_frTranslation);
  }

  @Override
  public void execute() {
    // System.out.println("aim all at speaker");
    updateDesiredVelocities();
    updatePositionsAndCompensatedTargets();

    ChassisSpeeds robotMovement = _frSpeeds.times(Constants.LOOP_PERIOD_S);
    Translation2d ffChassisMovement = new Translation2d(
        _currentPose.getX() + robotMovement.vxMetersPerSecond,
        _currentPose.getY() + robotMovement.vyMetersPerSecond
    );

    // do you take velo accel or both here?
    // THIS WAS WRONG AT THE START OF SVR, AIM FOR THE MIDDLE OF THE LEFT RIGHT
    // TARGETS INSTEAD
    // double goalYawRad = Drivetrain.getFaceTargetRad(_veloCompTarget,
    // ffChassisMovement);
    double left = Drivetrain.getFaceTargetRad(_leftVeloCompTarget, ffChassisMovement);
    double right = Drivetrain.getFaceTargetRad(_rightVeloCompTarget, ffChassisMovement);
    double goalYawRad = (left + AlgebraicUtils.placeInScopeRad(left, right)) / 2.0;
    double goalYawRadPerSec = (goalYawRad - _prevDesiredYawRad) / Constants.LOOP_PERIOD_S;

    _angleController.setSetpoint(_targetState.position);
    double pidRotationRadPS = _angleController.calculate(_currentYaw.getRadians());
    double netRotationalRadPS = pidRotationRadPS + _targetState.velocity;

    // Hi ryan, this is the one case where the new trap profile API is actually
    // better. -Eugene
    _targetState = _rotationProfile.calculate(
        Constants.LOOP_PERIOD_S,
        _targetState,
        new TrapezoidProfile.State(
            AlgebraicUtils.placeInScopeRad(_drivetrain.getYaw().getRadians(), goalYawRad),
            goalYawRadPerSec
        )
    );

    // TODO: Potentially filter on net movement rather than just rotational
    _drivetrain.fieldRelativeAntiscrubDrive(
        _xVelocityMPS,
        _yVelocityMPS,
        Math.abs(netRotationalRadPS) < 0.02 ? 0.0 : netRotationalRadPS,
        Drivetrain.ALLOWED_SCRUB,
        MAX_SPEED_MPS
    );
    _prevDesiredYawRad = goalYawRad;
    _prevNetRotationalRadPS = netRotationalRadPS;

    // target
    if (AlgebraicUtils.placeInScopeRot(
            _intake._pivotUnsafeMinRot, _intake.getPivot().getPivotPosition()
        )
        < _intake._pivotUnsafeMinRot) {
      _shooter.setPivotProfileFromTargetLinearized(ffChassisMovement.getDistance(_veloCompTarget));
    } else {
      _intake.getPivot().setPivotProfile(IntakePivotPositions.CLEAR_OF_SHOOTER.getRotations(), 0.0);
      _intake.getPivot().followLastProfile();
      _intake.getPivot().setFollowProfile(true);
    }
    _shooter.getPivot().followLastProfile();
    _lastDistance = _currentTranslation.getDistance(_veloCompTarget);
    _inTargetRange = _lastDistance <= MAX_TARGET_RANGE_M;
    _onPivotTarget = _shooter.isOnSOTFTargetWithMinRange(_lastDistance, MIN_ANGULAR_RANGE_ROT);
    _onYawTarget =
        drivetrainOnTarget(_drivetrain, _veloCompTarget, _leftVeloCompTarget, _rightVeloCompTarget);

    _filter.calculate(
        (AlgebraicUtils.placeInScopeRot(
             _shooter.getPivot().getPivotPosition(),
             _shooter.getTargetLaunchAngleFromTargetLinearizedRot(
                 _currentTranslation.getDistance(_veloCompTarget)
             )
         )
         - _shooter.getPivot().getPivotPosition())
    );

    if (RobotAltModes.isSOTFTuningMode) {
      System.out.println(
          "drivetrain on target " + _onYawTarget + " shooter on target " + _onPivotTarget
          + " flywheel at speed " + _shooter.isFlywheelAtSpeakerSpeed() + " desired shooter angle "
          + Units.rotationsToDegrees(AlgebraicUtils.placeInScopeRot(
              _shooter.getPivot().getPivotPosition(),
              _shooter.getTargetLaunchAngleFromTargetLinearizedRot(
                  _currentTranslation.getDistance(_veloCompTarget)
              )
          ))
          + " robot dist " + _currentTranslation.getDistance(_veloCompTarget)
      );
      _drivetrain._targetPositionEntry.setDouble(_targetState.position);
      _drivetrain._targetVelocityEntry.setDouble(_targetState.velocity);
      _drivetrain._curPositionEntry.setDouble(_drivetrain.getYaw().getRadians());
      _drivetrain._curVelocityEntry.setDouble(
          _drivetrain.getFieldRelativeSpeeds().omegaRadiansPerSecond
      );
    }
  }

  @Override
  public void end(boolean interrupted) {
    _drivetrain.resetDefaultTolerance();
    _drivetrain.fieldRelativeAntiscrubDrive(0.0, 0.0, 0.0);
    _onYawTarget = false;
    _onPivotTarget = false;
    if (RobotAltModes.isSOTFTuningMode) {
      System.out.println(
          "FILTERED DRIVETRAIN ERROR degs"
          + Units.radiansToDegrees(_filter.calculate(
              (AlgebraicUtils.placeInScopeRad(_currentYaw.getRadians(), _prevDesiredYawRad)
               - _currentYaw.getRadians())
          ))
      );
    }
    _shooter.getPivot().setPivotProfile(
        AlgebraicUtils.cppMod(_shooter.getPivot().getPivotPosition(), 1.0), 0.0
    );
    _shooter.getPivot().setFollowProfile(true);
  }

  public static boolean drivetrainOnTarget(
      Drivetrain drivetrain,
      Translation2d relativeTarget,
      Translation2d leftTarget,
      Translation2d rightTarget
  ) {
    return drivetrainOnTarget(
        drivetrain.getPose().getTranslation(),
        drivetrain.getYaw(),
        relativeTarget,
        leftTarget,
        rightTarget
    );
  }

  public static boolean drivetrainOnTarget(
      Translation2d currentLocation,
      Rotation2d yaw,
      Translation2d relativeTarget,
      Translation2d leftTarget,
      Translation2d rightTarget
  ) {
    double midTargetRad = Drivetrain.getFaceTargetRad(relativeTarget, currentLocation);
    double currentYawRad = yaw.getRadians();
    // place the target +/- 180 degrees of the current robot yaw
    double midTargetInCurrentRangeRad = AlgebraicUtils.placeInScopeRad(currentYawRad, midTargetRad);

    // find target robot angles (yaw) for left and right edges of the speaker
    double leftTargetRad = Drivetrain.getFaceTargetRad(leftTarget, currentLocation);
    double rightTargetRad = Drivetrain.getFaceTargetRad(rightTarget, currentLocation);

    // double[] yawTolerancesDeg =
    // computeYawToleranceDeg(drivetrain, relativeTarget, midTargetInCurrentRange);
    // System.out.println("Tolerances: " + yawTolerancesDeg[0] + " " +
    // yawTolerancesDeg[1]);
    // double yawLeftBoundDeg = midTargetInCurrentRange - yawTolerancesDeg[0];
    // double yawRightBoundDeg = midTargetInCurrentRange + yawTolerancesDeg[1];
    return currentYawRad
        <= AlgebraicUtils.placeInScopeRad(midTargetInCurrentRangeRad, leftTargetRad)
        && currentYawRad
        >= AlgebraicUtils.placeInScopeRad(midTargetInCurrentRangeRad, rightTargetRad);
  }

  private void updateDesiredVelocities() {
    double joystickX = _controlboard.getDriveX();
    double joystickY = _controlboard.getDriveY();

    double xyNet = Math.hypot(joystickX, joystickY);

    _xVelocityMPS = InputUtils.scaleJoystickXMPS(
        joystickX, xyNet, _drivetrain._translationalLimitsM.maxVelocity
    );
    _yVelocityMPS = InputUtils.scaleJoystickYMPS(
        joystickY, xyNet, _drivetrain._translationalLimitsM.maxVelocity
    );
  }

  public void setChoreoAimDriverVelocities(double xMPS, double yMPS) {
    _xVelocityMPS = xMPS;
    _yVelocityMPS = yMPS;
  }

  // private double binarySearchFor

  // current velocity, target, and shot speed then find T within a given range
}
