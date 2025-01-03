package frc.robot.commands.base;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.enums.ScoreTargets;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.InputUtils;
import tagalong.measurements.AlliancePose3d;

public class FaceAllianceTargetCommand extends Command {
  protected final Drivetrain _drivetrain;
  public static final Controlboard _controlboard = Controlboard.get();
  protected final AlliancePose3d _allianceTargets;
  protected Pose2d _targetPose;
  protected Translation2d _target;
  protected boolean _velocityCompensation;

  // private final double _xToleranceM;
  // private final double _yToleranceM;
  // private final Rotation2d _thetaTolerance;

  public FaceAllianceTargetCommand(
      Drivetrain drivetrain, ScoreTargets targets, boolean velocityCompensation
  ) {
    this(drivetrain, targets);
    _velocityCompensation = velocityCompensation;
  }

  public FaceAllianceTargetCommand(Drivetrain drivetrain, ScoreTargets targets) {
    _drivetrain = drivetrain;
    _allianceTargets = targets.target;
    addRequirements(drivetrain);
  }

  public FaceAllianceTargetCommand(Drivetrain drivetrain, AlliancePose3d targets) {
    _drivetrain = drivetrain;
    _allianceTargets = targets;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    // _drivetrain.setTolerance(_xToleranceM, _yToleranceM, _thetaTolerance);
    _targetPose = _allianceTargets.get(Controlboard.isRedAlliance()).toPose2d();
    _target = _targetPose.getTranslation();
    _drivetrain.setFaceTarget(_target);
  }

  @Override
  public void execute() {
    if (_velocityCompensation) {
      _target = getCompensatedGoalLocation2d(_targetPose, _drivetrain.getFieldRelativeSpeeds());
    }

    double joystickX = _controlboard.getDriveX();
    double joystickY = _controlboard.getDriveY();
    double xyNet = Math.hypot(joystickX, joystickY);
    _drivetrain.faceTargetMode(
        InputUtils.scaleJoystickXMPS(
            joystickX, xyNet, _drivetrain._translationalLimitsM.maxVelocity
        ),
        InputUtils.scaleJoystickYMPS(
            joystickY, xyNet, _drivetrain._translationalLimitsM.maxVelocity
        )
    );
  }

  @Override
  public void end(boolean interrupted) {
    _drivetrain.resetDefaultTolerance();
    _drivetrain.fieldRelativeAntiscrubDrive(0.0, 0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private static Translation2d getCompensatedGoalLocation2d(Pose2d target, ChassisSpeeds speeds) {
    return new Translation2d(
        target.getX() - speeds.vxMetersPerSecond * Constants.LOOP_PERIOD_S,
        target.getY() - speeds.vyMetersPerSecond * Constants.LOOP_PERIOD_S
    );
  }
}
