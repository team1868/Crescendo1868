package frc.robot.commands.base;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.enums.VisionModes;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;

public class VisionCenterPixelCommand extends Command {
  private final Drivetrain _drivetrain;
  public static final Controlboard _controlboard = Controlboard.get();
  private final VisionModes _visionMode;
  private VisionModes _prevVisionMode;

  public VisionCenterPixelCommand(Drivetrain drivetrain, VisionModes visionMode) {
    _drivetrain = drivetrain;
    _visionMode = visionMode;
  }

  @Override
  public void initialize() {
    _prevVisionMode = _drivetrain.getVisionMode();
    _drivetrain.setVisionMode(_visionMode);
  }

  @Override
  public void execute() {
    double _centerOffsetX = _drivetrain.getCenterOffsetX(_visionMode);
    _drivetrain.fieldRelativeAntiscrubDrive(
        _controlboard.getDriveX(),
        Double.isInfinite(_centerOffsetX)
            ? _controlboard.getDriveY()
            : _drivetrain.getVisionCenterPixelController(_centerOffsetX),
        0.0
    );
  }

  @Override
  public void end(boolean interrupted) {
    _drivetrain.setVisionMode(_prevVisionMode);
  }
}
