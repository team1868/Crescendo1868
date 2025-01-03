package frc.robot.commands.base;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.enums.LedModes;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Leds;
import tagalong.measurements.AlliancePose2d;

public class GoToPoseLedCommand extends GoToPoseCommand {
  private LedVariableSpeedCommand _wrappedLedCommand;
  private Translation2d _translationalTarget;
  private double _initialDistance;

  protected GoToPoseLedCommand(
      PoseType type,
      Drivetrain drivetrain,
      Leds leds,
      AlliancePose2d alliance,
      Pose2d absolute,
      Transform2d relative,
      double xToleranceM,
      double yToleranceM,
      Rotation2d thetaTolerance,
      LedModes animation
  ) {
    super(type, drivetrain, alliance, absolute, relative, xToleranceM, yToleranceM, thetaTolerance);
    _wrappedLedCommand = new LedVariableSpeedCommand(leds, animation, this::getSpeed);
  }

  public GoToPoseLedCommand(
      Drivetrain drivetrain,
      Leds leds,
      AlliancePose2d pose,
      double xToleranceM,
      double yToleranceM,
      Rotation2d thetaTolerance,
      LedModes animation
  ) {
    super(
        PoseType.ALLIANCE, drivetrain, pose, null, null, xToleranceM, yToleranceM, thetaTolerance
    );
    _wrappedLedCommand = new LedVariableSpeedCommand(leds, animation, this::getSpeed);
  }

  public GoToPoseLedCommand(
      Drivetrain drivetrain,
      Leds leds,
      Pose2d pose,
      double xToleranceM,
      double yToleranceM,
      Rotation2d thetaTolerance,
      LedModes animation
  ) {
    super(
        PoseType.ALLIANCE, drivetrain, null, pose, null, xToleranceM, yToleranceM, thetaTolerance
    );
    _wrappedLedCommand = new LedVariableSpeedCommand(leds, animation, this::getSpeed);
  }

  public GoToPoseLedCommand(
      Drivetrain drivetrain,
      Leds leds,
      Transform2d relative,
      double xToleranceM,
      double yToleranceM,
      Rotation2d thetaTolerance,
      LedModes animation
  ) {
    super(
        PoseType.ALLIANCE,
        drivetrain,
        null,
        null,
        relative,
        xToleranceM,
        yToleranceM,
        thetaTolerance
    );
    _wrappedLedCommand = new LedVariableSpeedCommand(leds, animation, this::getSpeed);
  }

  public GoToPoseLedCommand(
      Drivetrain drivetrain, Leds leds, AlliancePose2d pose, LedModes animation
  ) {
    this(
        drivetrain,
        leds,
        pose,
        drivetrain._translationalToleranceM,
        drivetrain._translationalToleranceM,
        drivetrain._rotationalToleranceRot,
        animation
    );
  }

  public GoToPoseLedCommand(Drivetrain drivetrain, Leds leds, Pose2d pose, LedModes animation) {
    this(
        drivetrain,
        leds,
        pose,
        drivetrain._translationalToleranceM,
        drivetrain._translationalToleranceM,
        drivetrain._rotationalToleranceRot,
        animation
    );
  }

  public GoToPoseLedCommand(
      Drivetrain drivetrain, Leds leds, Transform2d relative, LedModes animation
  ) {
    this(
        drivetrain,
        leds,
        relative,
        drivetrain._translationalToleranceM,
        drivetrain._translationalToleranceM,
        drivetrain._rotationalToleranceRot,
        animation
    );
  }

  public double getSpeed() {
    return 1.0
        - (_drivetrain.getPose().getTranslation().getDistance(_translationalTarget)
           / _initialDistance);
  }

  @Override
  public void initialize() {
    super.initialize();
    _wrappedLedCommand.execute();

    _translationalTarget = _target.getTranslation();
    _initialDistance = _drivetrain.getPose().getTranslation().getDistance(_translationalTarget);
  }

  @Override
  public void execute() {
    super.execute();
    _wrappedLedCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    _wrappedLedCommand.end(interrupted);
  }
}
