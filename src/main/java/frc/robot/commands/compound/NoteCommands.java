package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.base.FaceAllianceTargetCommand;
import frc.robot.commands.base.RollerRotateXCommand;
import frc.robot.commands.complex.SOTFCommand;
import frc.robot.commands.complex.SafePivotToCommand;
import frc.robot.constants.enums.IntakePivotPositions;
import frc.robot.constants.enums.LedColors;
import frc.robot.constants.enums.LedModes;
import frc.robot.constants.enums.ScoreTargets;
import frc.robot.constants.enums.ShooterPivotPositions;
import frc.robot.constants.enums.ShooterRollerDistances;
import frc.robot.parsers.json.IntakeConfJson;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShotConstants;

public class NoteCommands {
  public static Command ejectNoteCommand(Intake intake, Shooter shooter, Leds leds) {
    return Commands
        .deadline(
            Commands.waitUntil(() -> !(intake.isPieceInIntake() || shooter.isPieceInChute()))
                .andThen(Commands.waitSeconds(ShotConstants.EJECT_WAIT_S)),
            Commands.parallel(
                new SafePivotToCommand(
                    intake,
                    shooter,
                    IntakePivotPositions.EJECT_NOTE,
                    ShooterPivotPositions.EJECT_NOTE,
                    true
                ),
                shooter.getRoller().startEndRollerRPSWithFFCommand(ShotConstants.ROLLER_EJECT_RPS),
                intake.getRoller().startEndRollerRPSWithFFCommand(IntakeConstants.ROLLER_EJECT_RPS)
            )
        )
        .andThen(IntakeCommands.stowCommand(intake, shooter))
        .beforeStarting(() -> { leds.unsafeSetLEDMode(LedModes.YELLOW_SOLID, 0); })
        .finallyDo(() -> {
          leds.unsafeSetLEDMode(
              shooter.isPieceInChute() ? LedModes.GREEN_SOLID : LedModes.RED_SOLID, 0
          );
        });
  }

  public static Command loadNoteForSpeakerCommand(
      Intake intake, Shooter shooter, double lowerToleranceRot, double upperToleranceRot
  ) {
    return Commands.sequence(
        new SafePivotToCommand(
            intake, shooter, IntakePivotPositions.PASS_OFF, ShooterPivotPositions.PASS_OFF, true
        ),
        Commands.parallel(
            intake.getRoller().startEndRollerRPSWithFFCommand(Intake.IntakeConstants.ROLLER_IN_RPS),
            shooter.getRoller().startEndRollerRPSWithFFCommand(Shooter.ShotConstants.ROLLER_IN_RPS)
        ),
        // .until(() -> shooter.isPieceInChute()), break beam
        Commands.waitSeconds(5.0),
        new RollerRotateXCommand<Shooter>(
            shooter,
            ShooterRollerDistances.MOVE_NOTE_BACK.getRotations(),
            lowerToleranceRot,
            upperToleranceRot
        )
    );
  }

  public static Command loadNoteForTrapCommand(
      Shooter shooter, double lowerTolerance, double upperTolerance
  ) {
    return Commands.sequence(
        shooter.getRoller()
            .startEndRollerRPSWithFFCommand(Shooter.ShotConstants.ROLLER_OUT_RPS)
            .until(() -> !shooter.isPieceInChute()),
        new RollerRotateXCommand<Shooter>(
            shooter, ShooterRollerDistances.TRAP_PREP, lowerTolerance, upperTolerance
        )
    );
  }

  public static Command launchAtAmpCommand(
      Drivetrain drive, Shooter shooter, Intake intake, Leds leds
  ) {
    return Commands
        .sequence(
            Commands.waitUntil(() -> shooter.isAtAmpLaunchSpeed()),
            Commands.waitUntil(() -> {
              //   System.out.println("not on target");
              return SOTFCommand.drivetrainOnTarget(
                  drive,
                  ScoreTargets.AMP_LAUNCH_TARGET.getTranslation2d(Controlboard.isRedAlliance()),
                  ScoreTargets.AMP_LEFT_LAUNCH_TARGET.getTranslation2d(Controlboard.isRedAlliance()
                  ),
                  ScoreTargets.AMP_RIGHT_LAUNCH_TARGET.getTranslation2d(Controlboard.isRedAlliance()
                  )
              );
            }),
            Commands
                .sequence(
                    Commands.waitUntil(() -> !shooter.isPieceInChute()),
                    Commands.waitSeconds(Shooter.ShotConstants.FLYWHEEL_WAIT_S)
                )
                .deadlineWith(shooter.getRoller().startEndRollerRPSWithFFCommand(
                    Shooter.ShotConstants.ROLLER_IN_RPS
                ))
        )
        .deadlineWith(
            new FaceAllianceTargetCommand(drive, ScoreTargets.AMP_LAUNCH_TARGET),
            Commands.run(
                ()
                    -> shooter.setAmpLaunchVelocity(drive.getPose().getTranslation().getDistance(
                        ScoreTargets.AMP_LAUNCH_TARGET.getTranslation2d(Controlboard.isRedAlliance()
                        )
                    ))
            ),
            Commands
                .parallel(
                    intake.getRoller().startEndRollerRPSWithFFCommand(IntakeConstants.ROLLER_IN_RPS
                    ),
                    Commands.either(
                        new SafePivotToCommand(
                            intake,
                            shooter,
                            IntakePivotPositions.RED_GROUND_INTAKE,
                            ShooterPivotPositions.PASS_OFF,
                            true
                        ),
                        new SafePivotToCommand(
                            intake,
                            shooter,
                            IntakePivotPositions.BLUE_GROUND_INTAKE,
                            ShooterPivotPositions.PASS_OFF,
                            true
                        ),
                        Controlboard::isRedAlliance
                    )
                )
                .until(() -> !Controlboard.driverLaunchAtAmpButton().getAsBoolean())
                .andThen(new SafePivotToCommand(
                    intake, shooter, IntakePivotPositions.STOW, ShooterPivotPositions.PASS_OFF, true
                ))
        )
        .beforeStarting(() -> {
          shooter._ampTargetSpeed = 90.0;
          leds.unsafeSetLEDMode(LedModes.BLUE_COLOR_FLOW, 0);
        })
        .finallyDo(() -> {
          shooter.getFlywheel().setFlywheelPower(0.0);
          leds.unsafeSetLEDMode(
              shooter.isPieceInChute() ? LedModes.GREEN_SOLID : LedModes.RED_SOLID, 0
          );
        });
  }
}
