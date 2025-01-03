package frc.robot.commands.compound;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.base.ElevatorRaiseToCommand;
import frc.robot.commands.base.GoToPoseCommand;
import frc.robot.commands.base.GoToPoseLedCommand;
import frc.robot.commands.base.LedCommand;
import frc.robot.commands.base.LedVariableSpeedCommand;
import frc.robot.commands.base.PivotToCommand;
import frc.robot.commands.base.TeleopSwerveCommand;
import frc.robot.commands.complex.SafePivotToCommand;
import frc.robot.constants.enums.ClimberPivotPositions;
import frc.robot.constants.enums.ClimberPositions;
import frc.robot.constants.enums.DriveModes;
import frc.robot.constants.enums.HookPositions;
import frc.robot.constants.enums.IntakePivotPositions;
import frc.robot.constants.enums.LedModes;
import frc.robot.constants.enums.ShooterPivotPositions;
import frc.robot.constants.enums.StaticTargets;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ElevatorConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShotConstants;
import java.util.HashMap;

public class ClimbCommands {
  private static HashMap<Integer, Command> _stageCommandMap;
  private static final double _climbSettleTime = 3.0;

  public static Command climbCommand(
      Drivetrain drivetrain, Intake intake, Shooter shooter, Climber climber, Leds leds
  ) {
    return climbCommand(
        drivetrain,
        intake,
        shooter,
        climber,
        leds,
        drivetrain._translationalToleranceM,
        drivetrain._translationalToleranceM,
        drivetrain._rotationalToleranceRot,
        drivetrain._translationalToleranceM,
        drivetrain._translationalToleranceM,
        drivetrain._rotationalToleranceRot
    );
  }

  // TODO: tuning
  public static Command climbCommand(
      Drivetrain drivetrain,
      Intake intake,
      Shooter shooter,
      Climber climber,
      Leds leds,
      double xInitialTolerance,
      double yInitialTolerance,
      Rotation2d thetaInitialTolerance,
      double xFinalTolerance,
      double yFinalTolerance,
      Rotation2d thetaFinalTolerance
  ) {
    return Commands.sequence(
        prepToClimbCommand(
            drivetrain,
            climber,
            intake,
            shooter,
            leds,
            xInitialTolerance,
            yInitialTolerance,
            thetaInitialTolerance
        ),
        engageChainCommand(
            drivetrain, climber, leds, xFinalTolerance, yFinalTolerance, thetaFinalTolerance
        ),
        elevateCommand(climber, leds),
        trapScoreCommand(climber, intake, shooter, leds)
    );
  }

  // TODO: tuning
  private static Command prepToClimbCommand(
      Drivetrain drivetrain,
      Climber climber,
      Intake intake,
      Shooter shooter,
      Leds leds,
      double xInitialTolerance,
      double yInitialTolerance,
      Rotation2d thetaInitialTolerance
  ) {
    if (_stageCommandMap == null) {
      _stageCommandMap = new HashMap<Integer, Command>();
      _stageCommandMap.put(
          1,
          new GoToPoseLedCommand(
              drivetrain,
              leds,
              StaticTargets.STAGE_ALIGNED_1.target,
              xInitialTolerance,
              yInitialTolerance,
              thetaInitialTolerance,
              LedModes.RED_SOLID
          )
      );
      _stageCommandMap.put(
          2,
          new GoToPoseLedCommand(
              drivetrain,
              leds,
              StaticTargets.STAGE_ALIGNED_2.target,
              xInitialTolerance,
              yInitialTolerance,
              thetaInitialTolerance,
              LedModes.RED_SOLID
          )
      );
      _stageCommandMap.put(
          3,
          new GoToPoseLedCommand(
              drivetrain,
              leds,
              StaticTargets.STAGE_ALIGNED_3.target,
              xInitialTolerance,
              yInitialTolerance,
              thetaInitialTolerance,
              LedModes.RED_SOLID
          )
      );
    }

    return Commands.sequence(
        new TeleopSwerveCommand(drivetrain, DriveModes.FIELD_RELATIVE_ANTISCRUB)
            .until(() -> drivetrain.seesValidStageTarget()),
        // TODO: Move the intake and shooter into the "safe" positions--added below
        Commands.parallel(
            new PivotToCommand(climber, ClimberPivotPositions.CLIMB_PREP, true),
            new SafePivotToCommand(
                intake,
                shooter,
                IntakePivotPositions.CLIMB_PREP,
                ShooterPivotPositions.CLIMB_PREP,
                true
            )
        ),
        Commands.select(_stageCommandMap, () -> drivetrain.getStageTargetID())
    );
  }

  public static Command engageChainCommand(
      Drivetrain drivetrain,
      Climber climber,
      Leds leds,
      double xFinalTolerance,
      double yFinalTolerance,
      Rotation2d thetaFinalTolerance
  ) {
    return Commands
        .sequence(
            new GoToPoseCommand(
                drivetrain,
                drivetrain.getPose().transformBy(new Transform2d()), // TODO: add transformation
                xFinalTolerance,
                yFinalTolerance,
                thetaFinalTolerance
            ),
            // TODO: vision to align with target centrally
            Commands.waitUntil(() -> climber.isReadyToClimb())
        )
        .deadlineWith(new LedCommand(leds, LedModes.LARSON_PINK));
  }

  public static Command elevateCommand(Climber climber, Leds leds) {
    return new PivotToCommand(climber, ClimberPivotPositions.CLIMB, true)
        .deadlineWith(new LedVariableSpeedCommand(
            leds, LedModes.LARSON_GREEN, climber.getElevator()::getPercentOfSpeedLEDSpeed
        ));
  }

  public static Command trapScoreCommand(
      Climber climber, Intake intake, Shooter shooter, Leds leds
  ) {
    return Commands
        .sequence(
            StaticScoreCommands.handOffCommand(climber, intake, shooter, leds),
            new ElevatorRaiseToCommand<Climber>(
                climber, ClimberPositions.TRAP_INTERMEDIATE, ElevatorConstants.ELEVATOR_TRAP_MPS
            ),
            Commands.parallel(
                climber.getRoller()
                    .startEndRollerRPSWithFFCommand(ShotConstants.ROLLER_TRAP_RPS)
                    .withTimeout(ShotConstants.TRAP_TIMEOUT_S),
                new ElevatorRaiseToCommand<Climber>(
                    climber, ClimberPositions.TRAP_FINAL, ElevatorConstants.ELEVATOR_TRAP_MPS
                )
            )
        )
        .deadlineWith(new LedVariableSpeedCommand(
            leds, LedModes.LARSON_ORANGE, shooter::getPercentOfSpeedLEDSpeedClimb
        ));
  }
}
