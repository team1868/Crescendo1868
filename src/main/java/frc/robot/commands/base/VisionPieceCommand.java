package frc.robot.commands.base;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.enums.VisionModes;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;

public class VisionPieceCommand extends Command {
  private final Drivetrain _drivetrain;
  private Controlboard _controlboard;
  private final double _xToleranceM;
  private final double _yToleranceM;
  private final Rotation2d _thetaTolerance;
  private Pose2d _target;
  private boolean _pieceSeen;
  private VisionModes _prevVisionMode;

  public VisionPieceCommand(Drivetrain drivetrain, Controlboard controlboard) {
    this(
        drivetrain,
        controlboard,
        drivetrain._translationalToleranceM,
        drivetrain._translationalToleranceM,
        drivetrain._rotationalToleranceRot
    );
  }

  public VisionPieceCommand(
      Drivetrain drivetrain,
      Controlboard controlboard,
      double xToleranceM,
      double yToleranceM,
      Rotation2d thetaTolerance
  ) {
    _drivetrain = drivetrain;
    _controlboard = controlboard;
    _xToleranceM = xToleranceM;
    _yToleranceM = yToleranceM;
    _thetaTolerance = thetaTolerance;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    _prevVisionMode = _drivetrain.getVisionMode();
    _drivetrain.setVisionMode(VisionModes.GROUND_INTAKE);
    Pose2d piecePose = _drivetrain.getPiecePose();
    while (piecePose == null) {
      piecePose = _drivetrain.getPiecePose();
      _drivetrain.fieldRelativeAntiscrubDrive(
          _controlboard.getDriveX(), _controlboard.getDriveY(), _controlboard.getRotX()
      );
      _pieceSeen = false;
    }
    _drivetrain.setTolerance(_xToleranceM, _yToleranceM, _thetaTolerance);
    _target = new Pose2d(
        _drivetrain.getPose().getTranslation().plus(piecePose.getTranslation()),
        _drivetrain.getYaw().minus(piecePose.getRotation())
    );
    _pieceSeen = true;
    _drivetrain.setStaticTarget(_target);
  }

  @Override
  public void execute() {
    if (_pieceSeen)
      _drivetrain.chaseStaticTargetDrive();
  }

  @Override
  public void end(boolean interrupted) {
    _drivetrain.resetDefaultTolerance();
    _drivetrain.setVisionMode(_prevVisionMode);
    _drivetrain.fieldRelativeAntiscrubDrive(0.0, 0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    return _drivetrain.inRange();
  }
}
