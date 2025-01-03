package frc.robot.commands.complex;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.enums.ScoreTargets;
import frc.robot.constants.enums.ScoreTargets.SpeakerConstants;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShotConstants;

// use equation & not comp for TTI & lock speed only after shot
public class ScoreSpeakerOnTheFlyCommand4 extends Command {
  public static final Controlboard _controlboard = Controlboard.get();
  private final Drivetrain _drivetrain;
  private final Shooter _shooter;

  public boolean _startedShooting;

  private Timer _timer = new Timer();
  private Translation3d _target3d;
  private Translation2d _target2d;
  private ChassisSpeeds _desiredFieldRelativeSpeeds;

  private double[] _yawToleranceLeftRightDeg = new double[2];

  public ScoreSpeakerOnTheFlyCommand4(Drivetrain drivetrain, Shooter shooter) {
    _drivetrain = drivetrain;
    _shooter = shooter;
    // addRequirements(_drivetrain, _shooter);
  }

  @Override
  public void initialize() {
    // _shooter.setFlywheelPower(ShotConstants.FLYWHEEL_SPEAKER_SPEED_RPS);
    _startedShooting = false;
    _timer.reset();
    _target3d = ScoreTargets.SPEAKER_BOTTOM.getTranslation3d(Controlboard.isRedAlliance());
    _target2d = _target3d.toTranslation2d();
    _desiredFieldRelativeSpeeds = _drivetrain.getFieldRelativeSpeeds();

    var pose = new Pose2d(16.54, 5.55, Rotation2d.fromDegrees(0.0));
    _drivetrain.setPose(pose, Rotation2d.fromDegrees(0.0));

    // compensate for TTO given original velocity
    double TTO = getTravelTimeOutside(
        _drivetrain.getPose().getTranslation().getDistance(_target2d),
        Units.rotationsToDegrees(_shooter.getPivot().getPivotAbsolutePositionRot())
    );
    Translation2d ttoMovement = getNetDistance(_desiredFieldRelativeSpeeds, TTO);

    // velocity adjusted aim point
    _drivetrain.setFaceTarget(_target2d); //.minus(ttoMovement));
  }

  @Override
  public void end(boolean interrupted) {
    _shooter.getPivot().setHoldPivotPosition(true);
    _shooter.getRoller().setRollerPower(0.0);
    _shooter.getFlywheel().setFlywheelPower(0.0);
  }

  @Override
  public boolean isFinished() {
    return !_shooter.isPieceInChute() && _timer.get() > ShotConstants.FLYWHEEL_WAIT_S;
  }

  @Override
  public void execute() {
    var drivePose = _drivetrain.getPose();

    // // Compensated for TTO
    // double TTO = getTravelTimeOutside(
    //     drivePose.getTranslation().getDistance(_target2d),
    //     Units.rotationsToDegrees(_shooter.getPivotAbsolutePositionRot())
    // );
    // // Translation2d ttoMovement = getNetDistance(_desiredFieldRelativeSpeeds, TTO);

    // Translation2d velocityCompensatedRobot = drivePose.getTranslation(); //.plus(ttoMovement);

    // _shooter.setPivotProfileFromTargetPolynomial(velocityCompensatedRobot, _target2d);
    // _shooter.followLastProfile();

    _drivetrain.faceTargetMode(
        _desiredFieldRelativeSpeeds.vxMetersPerSecond,
        _desiredFieldRelativeSpeeds.vyMetersPerSecond,
        _drivetrain._translationalLimitsM.maxVelocity,
        _drivetrain._angularMaxVeloRad
    );
    // double netDistance = velocityCompensatedRobot.getDistance(_target2d);
    // Translation2d relativeTarget = _target2d; //.minus(ttoMovement);
    // System.out.println("relative speaker target " + relativeTarget);

    // if (_startedShooting) {
    //   if (!_shooter.isPieceInChute()) {
    //     _timer.reset();
    //   }
    // } else {
    //   _desiredFieldRelativeSpeeds = _drivetrain.getFieldRelativeSpeeds();

    //   if (_shooter.isFlywheelAtSpeakerSpeed()
    //       && drivetrainOnTarget(drivePose, netDistance, relativeTarget)
    //       && pivotOnTarget(
    //           drivePose.getTranslation(), _shooter.getPivotAbsolutePositionRot(), relativeTarget
    //       )) {
    //     _startedShooting = true;
    //     // TODO update this after get set rpm logic in
    //     // _shooter.setRollerVelocityControl(ShotConstants.ROLLER_SPEAKER_RPS, true);
    //   }
    // }
  }

  private Translation2d getNetDistance(ChassisSpeeds frSpeeds, double travelTime) {
    return new Translation2d(frSpeeds.vxMetersPerSecond, frSpeeds.vyMetersPerSecond)
        .times(travelTime);
  }

  private boolean pivotOnTargetPolynomial(
      Translation2d robot, double pivotRot, Translation2d relativeTarget
  ) {
    double distance = robot.getDistance(relativeTarget);
    return pivotRot >= _shooter.getMinLaunchAngleFromTargetPolynomialRot(distance)
        && pivotRot <= _shooter.getMaxLaunchAngleFromTargetPolynomialRot(distance);
  }

  private boolean drivetrainOnTarget(
      Pose2d pose, double netDistance, Translation2d relativeTarget
  ) {
    computeYawToleranceDeg(pose, relativeTarget);
    double midYaw = _drivetrain.getFaceTargetRad();
    double yawLeftBoundDeg = midYaw - _yawToleranceLeftRightDeg[0];
    double yawRightBoundDeg = midYaw + _yawToleranceLeftRightDeg[1];
    double yawDeg = pose.getRotation().getDegrees();

    return yawDeg >= yawLeftBoundDeg && yawDeg <= yawRightBoundDeg;
  }

  private void computeYawToleranceDeg(Pose2d pose, Translation2d target) {
    // assume robot facing target
    // x, y distance between robot and speaker (meters)
    double deltaX = pose.getX() - target.getX();
    double deltaY = pose.getY() - target.getY();

    // c: half speaker width minus tolerance
    double c = (SpeakerConstants.WIDTH_M / 2.0) - SpeakerConstants.SPEAKER_EDGE_TOLERANCE_M;
    double m = deltaX - c;

    // theta: theta of robot facing speaker off of y
    double theta = Math.atan((deltaX + c) / deltaY);
    double yaw = _drivetrain.getFaceTargetRad();
    double k = Math.atan(m / deltaY);
    double left = Math.abs(theta - yaw);
    double right = Math.abs(yaw - k);

    if (deltaX == 0) {
      _yawToleranceLeftRightDeg[0] = -left;
      _yawToleranceLeftRightDeg[1] = right;
    } else if (deltaX < 0) {
      _yawToleranceLeftRightDeg[0] = -left;
      _yawToleranceLeftRightDeg[1] = -right;
    } else {
      _yawToleranceLeftRightDeg[0] = left;
      _yawToleranceLeftRightDeg[1] = right;
    }
  }

  private double getTravelTimeOutside(double distFromTargetM, double desiredPivotAngleRad) {
    return distFromTargetM / (ShotConstants.NOTE_SPEAKER_INIT_MPS * Math.cos(desiredPivotAngleRad));
  }
}
