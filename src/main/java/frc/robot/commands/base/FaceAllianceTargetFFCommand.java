package frc.robot.commands.base;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotAltModes;
import frc.robot.constants.enums.ScoreTargets;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.InputUtils;
import tagalong.math.AlgebraicUtils;
import tagalong.math.GeometricUtils;

public class FaceAllianceTargetFFCommand extends FaceAllianceTargetCommand {
  protected ChassisSpeeds _frSpeeds;
  protected Pose2d _currentPose;
  protected double _prevDesiredYawRad;
  protected PIDController _angleController;
  protected Rotation2d _currentYaw;
  public static boolean _onTarget;
  protected TrapezoidProfile _rotationProfile;
  protected double _prevNetRotationalRadPS;
  protected TrapezoidProfile.State _targetState = new TrapezoidProfile.State();
  protected TrapezoidProfile.State _dynamicGoalState = new TrapezoidProfile.State();
  protected ScoreTargets _leftTarget;
  protected ScoreTargets _rightTarget;
  private LinearFilter _filter = LinearFilter.movingAverage(50);

  public static final Controlboard _controlboard = Controlboard.get();

  public FaceAllianceTargetFFCommand(
      Drivetrain drivetrain, ScoreTargets targets, ScoreTargets leftTarget, ScoreTargets rightTarget
  ) {
    super(drivetrain, targets.target);
    _angleController = new PIDController(0.8, 0.0, 0.0);
    _angleController.enableContinuousInput(0.0, 2 * Math.PI);
    _leftTarget = leftTarget;
    _rightTarget = rightTarget;
    _leftTarget = leftTarget;
    _rightTarget = rightTarget;
  }

  public FaceAllianceTargetFFCommand(Drivetrain drivetrain, ScoreTargets targets) {
    this(drivetrain, targets, ScoreTargets.SPEAKER_LEFT_BOTTOM, ScoreTargets.SPEAKER_RIGHT_BOTTOM);
  }

  @Override
  public void initialize() {
    super.initialize();
    _onTarget = false;
    _currentPose = _drivetrain.getPose();
    _currentYaw = _drivetrain.getYaw();
    _prevDesiredYawRad = Drivetrain.getDesiredYawRad(
        _target.getX(), _target.getY(), _currentPose.getX(), _currentPose.getY()
    );
    _angleController.setSetpoint(_prevDesiredYawRad);
    _rotationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
        _drivetrain._angularMaxVeloRad, _drivetrain._angularMaxAccelRad
    ));

    _targetState.position = _drivetrain.getYaw().getRadians();
    _targetState.velocity = 0.0;
  }

  @Override
  public void execute() {
    double joystickX = _controlboard.getDriveX();
    double joystickY = _controlboard.getDriveY();
    double xyNet = Math.hypot(joystickX, joystickY);
    double xVelocityMPS = InputUtils.scaleJoystickXMPS(
        joystickX, xyNet, _drivetrain._translationalLimitsM.maxVelocity
    );
    double yVelocityMPS = InputUtils.scaleJoystickYMPS(
        joystickY, xyNet, _drivetrain._translationalLimitsM.maxVelocity
    );

    _currentPose = _drivetrain.getPose();
    _frSpeeds = _drivetrain.getFieldRelativeSpeeds(); // human driver request instead of cur velo?
    _frSpeeds = _drivetrain.getFieldRelativeSpeeds(); // human driver request instead of cur velo?
    _currentYaw = _drivetrain.getYaw();

    ChassisSpeeds robotMovement = _frSpeeds.times(Constants.LOOP_PERIOD_S);

    // do you take velo accel or both here?
    double goalYawRad = Drivetrain.getDesiredYawRad(
        _target.getX(),
        _target.getY(),
        _currentPose.getX() + robotMovement.vxMetersPerSecond,
        _currentPose.getY() + robotMovement.vyMetersPerSecond
    );
    double goalYawRadPerSec = (goalYawRad - _prevDesiredYawRad) / Constants.LOOP_PERIOD_S;

    _angleController.setSetpoint(_targetState.position);
    double pidRotationRadPS = _angleController.calculate(_currentYaw.getRadians());

    double netRotationalRadPS = pidRotationRadPS + _targetState.velocity;

    // Hi ryan, this is the one case where the new trap profile API is actually
    // better. -Eugene
    AlgebraicUtils.placeInScopeRad(_drivetrain.getYaw().getRadians(), goalYawRad);
    _dynamicGoalState.velocity = goalYawRadPerSec;
    _targetState =
        _rotationProfile.calculate(Constants.LOOP_PERIOD_S, _targetState, _dynamicGoalState);

    // TODO: Potentially filter on net movement rather than just rotational
    _drivetrain.fieldRelativeAntiscrubDrive(
        xVelocityMPS,
        yVelocityMPS,
        Math.abs(netRotationalRadPS) < 0.02 ? 0.0 : netRotationalRadPS,
        Drivetrain.ALLOWED_SCRUB,
        _drivetrain._translationalLimitsM.maxVelocity
    );
    _prevDesiredYawRad = goalYawRad;
    _prevNetRotationalRadPS = netRotationalRadPS;

    _onTarget = drivetrainOnTarget(
        _drivetrain,
        _target,
        ScoreTargets.SPEAKER_LEFT_BOTTOM.getTranslation2d(Controlboard.get().isRedAlliance()),
        ScoreTargets.SPEAKER_RIGHT_BOTTOM.getTranslation2d(Controlboard.get().isRedAlliance())
    );

    if (RobotAltModes.isSOTFTuningMode) {
      System.out.println("drivetrain on target" + _onTarget);
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
    super.end(interrupted);
    _onTarget = false;
    if (RobotAltModes.isSOTFTuningMode) {
      System.out.println(
          "FILTERED DRIVETRAIN ERROR degs"
          + Units.radiansToDegrees(_filter.calculate(
              (AlgebraicUtils.placeInScopeRad(_currentYaw.getRadians(), _prevDesiredYawRad)
               - _currentYaw.getRadians())
              // )
          ))
      );
    }
  }

  private static boolean drivetrainOnTarget(
      Drivetrain drivetrain,
      Translation2d relativeTarget,
      Translation2d leftTarget,
      Translation2d rightTarget
  ) {
    double midTargetRad = drivetrain.getFaceTargetRad();
    double currentYawRad = drivetrain.getYaw().getRadians();
    // place the target +/- 180 degrees of the current robot yaw
    double midTargetInCurrentRangeRad = AlgebraicUtils.placeInScopeRad(currentYawRad, midTargetRad);

    // find target robot angles (yaw) for left and right edges of the speaker
    double leftTargetRad = drivetrain.getFaceTargetRad(leftTarget);
    double rightTargetRad = drivetrain.getFaceTargetRad(rightTarget);

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

  // private static double[] computeYawToleranceDeg(
  // Drivetrain drivetrain, Translation2d target, double midYawTargetDeg
  // ) {
  // // assume robot facing target
  // // x, y distance between robot and speaker (meters)
  // Pose2d pose = drivetrain.getPose();
  // double deltaXM = Math.abs(pose.getX() - target.getX());
  // double deltaYM = Math.abs(pose.getY() - target.getY());
  // System.out.println("x " + deltaXM + " y " + deltaYM);

  // // c: half speaker width minus tolerance
  // double mM = Math.abs(deltaYM - SPEAKER_WIDTH_MINUS_TOLERANCE_DIV2);
  // System.out.println("c " + SPEAKER_WIDTH_MINUS_TOLERANCE_DIV2 + " m " + mM);

  // // theta: theta of robot facing speaker off of y
  // double thetaRad = Math.atan((deltaYM + cM) / deltaXM);
  // double yawRad = Math.PI / 2.0 - drivetrain.getFaceTargetRad();
  // double kRad = Math.atan(mM / deltaXM);
  // System.out.println("theta " + thetaRad + " yaw " + yawRad + " k " + kRad);

  // double rightDeg;
  // double leftDeg;
  // double yM = pose.getY() - target.getY();
  // if (yM == 0) {
  // System.out.println(0);
  // var deg = Units.radiansToDegrees(Math.atan(cM / deltaXM));
  // return new double[] {deg, deg};
  // } else if (yM < 0) { // on right of target
  // System.out.println("<");
  // rightDeg = Units.radiansToDegrees(Math.abs(Math.PI / 2.0 - yawRad - kRad));
  // leftDeg =
  // Units.radiansToDegrees(Math.abs(thetaRad - Units.degreesToRadians(rightDeg) -
  // kRad));
  // return new double[] {leftDeg, rightDeg};
  // } else { // on left of target
  // System.out.println(">");
  // leftDeg = Units.radiansToDegrees(Math.abs(Math.PI / 2.0 - yawRad - kRad));
  // rightDeg =
  // Units.radiansToDegrees(Math.abs(thetaRad - Units.degreesToRadians(leftDeg) -
  // kRad));
  // return new double[] {leftDeg, rightDeg};
  // }
  // }
}
