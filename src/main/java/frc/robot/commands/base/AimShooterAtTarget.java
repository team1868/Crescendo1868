package frc.robot.commands.base;

import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.enums.ScoreTargets;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShotConstants;
import frc.robot.utils.AlliancePose3d;
import frc.robot.utils.GeometricUtils;

public class AimShooterAtTarget extends Command {
  public static final Controlboard _controlboard = Controlboard.get();
  public static boolean _onTarget;

  private final Shooter _shooter;
  private final Drivetrain _drivetrain;

  private Pose2d _currentPose;
  private Translation2d _currentTranslation;
  public double _goalPivotRot;
  public double _ffPivotVelocity;

  private LinearFilter filter = LinearFilter.movingAverage(50);

  // up trap limits
  // tune p higher

  private final AlliancePose3d _allianceTargets;
  private Pose2d _targetPose;
  private Translation2d _target2d, _compensatedTarget2d;
  //   private ChassisSpeeds _desiredFieldRelativeSpeeds;

  // protected Translation2d _target;
  protected final PositionVoltage _pivotTargetControl = new PositionVoltage(0.0);

  public AimShooterAtTarget(Drivetrain drivetrain, Shooter shooter, ScoreTargets targetPose) {
    _allianceTargets = targetPose.target;
    _drivetrain = drivetrain;
    _shooter = shooter;
    addRequirements(_shooter);
  }

  @Override
  public void initialize() {
    _onTarget = false;
    _targetPose = _allianceTargets.get(Controlboard.isRedAlliance()).toPose2d();
    _target2d = _targetPose.getTranslation();
    _shooter.getPivot().setHoldPivotPosition(false);

    // V3
    // _drivetrain.setFaceTarget(_target2d);
  }

  @Override
  public void execute() {
    _compensatedTarget2d =
        getCompensatedGoalLocation2d(_targetPose, _drivetrain.getFieldRelativeSpeeds());
    // todo safety verifications
    // BEWARE OF LIMITS

    // V3
    // _drivetrain.setFaceTarget(_target2d);
    // _frSpeeds = _drivetrain.getFieldRelativeSpeeds();
    // Translation2d compensatedGoalPosition =
    //     getCompensatedGoalLocation2d(_frSpeeds, Constants.LOOP_PERIOD_S);

    // V1 set and follow profile--simple
    _currentPose = _drivetrain.getPose();
    _currentTranslation = _currentPose.getTranslation();
    _shooter.setPivotProfileFromTargetPolynomial(_currentTranslation, _compensatedTarget2d);
    _shooter.getPivot().followLastProfile();
    _onTarget = _shooter.isOnSOTFTarget(_currentTranslation.getDistance(_target2d));

    // curFilter.calculate(_shooter.getPivot().getPivotPosition());
    filter.calculate((
        GeometricUtils.placeInClosestRot(
            _shooter.getPivot().getPivotPosition(),
            _shooter.getTargetLaunchAngleFromTargetPolynomialRot(
                _currentTranslation.getDistance(_compensatedTarget2d)
            )
        )
        - _shooter.getPivot().getPivotPosition()
        // )
    ));

    // filter.calculate(
    //     ShotConstants.FLYWHEEL_SPEAKER_SPEED_RPS
    //     - _flywheel.getFlywheelMotor().getVelocity().getValueAsDouble()
    // );

    // System.out.println(
    //     "error" +
    //     // "filter"
    //     // + filter.calculate(
    //     (GeometricUtils.placeInClosestRot(
    //          _shooter.getPivot().getPivotPosition(),
    //          _shooter.getTargetLaunchAngleFromTargetPolynomialRot(
    //              _currentTranslation.getDistance(_compensatedTarget2d)
    //          )
    //      )
    //      - _shooter.getPivot().getPivotPosition()
    //      // )
    //     )
    // );

    // System.out.println("shooter on target" + _onTarget);
    // System.out.println("following profile");

    // V2 -- Positional voltage with FF velocity based on current and goal
    // _currentPose = _drivetrain.getPose();
    // _goalPivotRot = Units.radiansToRotations(
    //     _shooter.getLaunchAngleFromTargetPhysicsRad(_currentPose.getTranslation(), _target3d)
    // );
    // _ffPivotVelocity =
    //     (_goalPivotRot - _shooter.getPivotAbsolutePositionRot()) / Constants.LOOP_PERIOD_S;
    // _shooter.getPivotMotor().setControl(_pivotTargetControl.withPosition(_goalPivotRot)
    //                                         .withVelocity(_ffPivotVelocity)
    //                                         .withFeedForward(_shooter.calculateFF(_ffPivotVelocity))
    // );
    // _shooter.setPivotProfile(_goalPivotRot, _ffPivotVelocity);
    // _shooter.followLastProfile();

    // todo safety verifications

    // // V3 -- V2 + use a chassis velocity compensated target
    // _frSpeeds = _drivetrain.getFieldRelativeSpeeds();
    // Translation3d compensatedGoalPosition =
    //     getCompensatedGoalLocation3d(_frSpeeds, Constants.LOOP_PERIOD_S);
    // _goalPivotRot = Units.radiansToRotations(
    //     _shooter.getLaunchAngleFromTargetPhysicsRad(_currentPose.getTranslation(),
    //     compensatedGoalPosition)
    // );
    // _ffPivotVelocity =
    //     (_goalPivotRot - _shooter.getPivotAbsolutePositionRot()) / Constants.LOOP_PERIOD_S;
    // _shooter.getPivotMotor().setControl(_pivotTargetControl.withPosition(_goalPivotRot)
    //                                         .withFeedForward(_shooter.calculateFF(_ffPivotVelocity))
    // );
    // _shooter.followLastProfile();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // System.out.println(
    //     "FILTERED SHOOTER PIVOT ERROR degs"
    //     + (Units.rotationsToDegrees(filter.calculate((
    //         GeometricUtils.placeInClosestRot(
    //             _shooter.getPivot().getPivotPosition(),
    //             _shooter.getTargetLaunchAngleFromTargetPolynomialRot(
    //                 _currentTranslation.getDistance(_compensatedTarget2d)
    //             )
    //         )
    //         - _shooter.getPivot().getPivotPosition()
    //         // )
    //     ))))
    // );
    // System.out.println(
    //     "FILTERED SHOOTER FLYWHEEL ERROR rots"
    //     + filter.calculate(
    //         ShotConstants.FLYWHEEL_SPEAKER_SPEED_RPS
    //         - _shooter.getFlywheel().getFlywheelMotor().getVelocity().getValueAsDouble()
    //     )
    // );
    _onTarget = false;
    _shooter.getPivot().setPivotProfile(_shooter.getPivot().getPivotAbsolutePositionRot(), 0.0);
    _shooter.getPivot().setHoldPivotPosition(true);
  }

  private static Translation2d getCompensatedGoalLocation2d(Pose2d target, ChassisSpeeds speeds) {
    return new Translation2d(
        target.getX() - (speeds.vxMetersPerSecond * Constants.LOOP_PERIOD_S),
        target.getY() - (speeds.vyMetersPerSecond * Constants.LOOP_PERIOD_S)
    ); // TODO are speeds field relative? bc will this work on blue side? eh should be
  }
}
