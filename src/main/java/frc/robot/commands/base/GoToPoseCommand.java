package frc.robot.commands.base;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.AlliancePose2d;

public class GoToPoseCommand extends Command {
  public static enum PoseType { ABSOLUTE, ALLIANCE, RELATIVE }
  protected final Drivetrain _drivetrain;
  protected final Transform2d _relative;
  protected final PoseType _type;
  protected final double _xToleranceM;
  protected final double _yToleranceM;
  protected final Rotation2d _thetaTolerance;
  protected final AlliancePose2d _allianceTarget;
  protected Pose2d _target;

  protected GoToPoseCommand(
      PoseType type,
      Drivetrain drivetrain,
      AlliancePose2d alliance,
      Pose2d absolute,
      Transform2d relative,
      double xToleranceM,
      double yToleranceM,
      Rotation2d thetaTolerance
  ) {
    _type = type;
    _relative = relative;
    _target = absolute;
    _allianceTarget = alliance;

    _drivetrain = drivetrain;
    _xToleranceM = xToleranceM;
    _yToleranceM = yToleranceM;
    _thetaTolerance = thetaTolerance;

    addRequirements(drivetrain);
  }

  public GoToPoseCommand(
      Drivetrain drivetrain,
      AlliancePose2d pose,
      double xToleranceM,
      double yToleranceM,
      Rotation2d thetaTolerance
  ) {
    this(PoseType.ALLIANCE, drivetrain, pose, null, null, xToleranceM, yToleranceM, thetaTolerance);
  }

  public GoToPoseCommand(
      Drivetrain drivetrain,
      Pose2d pose,
      double xToleranceM,
      double yToleranceM,
      Rotation2d thetaTolerance
  ) {
    this(PoseType.ALLIANCE, drivetrain, null, pose, null, xToleranceM, yToleranceM, thetaTolerance);
  }

  public GoToPoseCommand(
      Drivetrain drivetrain,
      Transform2d relative,
      double xToleranceM,
      double yToleranceM,
      Rotation2d thetaTolerance
  ) {
    this(
        PoseType.ALLIANCE,
        drivetrain,
        null,
        null,
        relative,
        xToleranceM,
        yToleranceM,
        thetaTolerance
    );
  }

  public GoToPoseCommand(Drivetrain drivetrain, AlliancePose2d pose) {
    this(
        drivetrain,
        pose,
        drivetrain._translationalToleranceM,
        drivetrain._translationalToleranceM,
        drivetrain._rotationalToleranceRot
    );
  }

  public GoToPoseCommand(Drivetrain drivetrain, Pose2d pose) {
    this(
        drivetrain,
        pose,
        drivetrain._translationalToleranceM,
        drivetrain._translationalToleranceM,
        drivetrain._rotationalToleranceRot
    );
  }

  public GoToPoseCommand(Drivetrain drivetrain, Transform2d relative) {
    this(
        drivetrain,
        relative,
        drivetrain._translationalToleranceM,
        drivetrain._translationalToleranceM,
        drivetrain._rotationalToleranceRot
    );
  }

  @Override
  public void initialize() {
    _drivetrain.setTolerance(_xToleranceM, _yToleranceM, _thetaTolerance);

    switch (_type) {
      case ALLIANCE:
        _target = _allianceTarget.get(Controlboard.get().isRedAlliance());
        break;
      case RELATIVE:
        _target = _drivetrain.getPose().transformBy(_relative);
        break;
      case ABSOLUTE:
      default:
    }

    _drivetrain.setStaticTarget(_target);
  }

  @Override
  public void execute() {
    _drivetrain.chaseStaticTargetDrive();
  }

  @Override
  public void end(boolean interrupted) {
    _drivetrain.resetDefaultTolerance();
    _drivetrain.fieldRelativeAntiscrubDrive(0.0, 0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    return _drivetrain.inRange();
  }
}
