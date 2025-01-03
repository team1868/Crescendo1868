package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.base.AimShooterAtTarget;
import frc.robot.commands.base.FaceAllianceTargetFFCommand;
import frc.robot.commands.base.LedVariableSpeedCommand;
import frc.robot.commands.base.TeleopSwerveCommand;
import frc.robot.commands.complex.SafePivotToCommand;
import frc.robot.constants.enums.DriveModes;
import frc.robot.constants.enums.IntakePivotPositions;
import frc.robot.constants.enums.LedModes;
import frc.robot.constants.enums.ScoreTargets;
import frc.robot.constants.enums.ShooterPivotPositions;
import frc.robot.constants.enums.ShooterRollerDistances;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShotConstants;
import tagalong.commands.base.PivotToCmd;
import tagalong.commands.base.RollXCmd;
import tagalong.math.AlgebraicUtils;
import tagalong.subsystems.micro.Roller;

public class AutonomousCommands {
  public static Command prepToScoreCommand(Drivetrain drivetrain, Shooter shooter, Intake intake) {
    return Commands.parallel(
        Commands.sequence(
            new PivotToCmd<Shooter>(
                shooter,
                ShooterPivotPositions.PASS_OFF, // TODO change to
                                                // current position
                true
            ),
            new AimShooterAtTarget(drivetrain, shooter, ScoreTargets.SPEAKER_BOTTOM)
        ),
        shooter.getRoller(ShooterConstants.FLYWHEEL_ID)
            .startEndRollerRPSWithFFCmd(ShotConstants.FLYWHEEL_SPEAKER_SPEED_RPS)
    );
  }

  public static Command scoreCommand(Shooter shooter) {
    return shooter.getRoller().setRollerRPSWithFFCmd(ShotConstants.ROLLER_SPEAKER_RPS);
  }

  public static Command postScoreCommand(Shooter shooter) {
    return Commands.sequence(
        Commands.waitUntil(() -> !shooter.isPieceInChute()),
        Commands.waitSeconds(ShotConstants.FLYWHEEL_WAIT_S),
        shooter.getRoller(ShooterConstants.FLYWHEEL_ID).setRollerRPSWithFFCmd(0.0),
        shooter.getRoller().setRollerRPSWithFFCmd(0.0)
    );
  }

  public static Command prepToIntakeCommand(Intake intake, Shooter shooter) {
    return Commands.either(
        Commands.parallel(
            new PivotToCmd<Intake>(intake, IntakePivotPositions.RED_GROUND_INTAKE, true),
            intake.getRoller().setRollerRPSWithFFCmd(IntakeConstants.ROLLER_IN_RPS)
        ),
        Commands.parallel(
            new PivotToCmd<Intake>(intake, IntakePivotPositions.BLUE_GROUND_INTAKE, true),
            intake.getRoller().setRollerRPSWithFFCmd(IntakeConstants.ROLLER_IN_RPS)
        ),
        Controlboard::isRedAlliance
    );
  }

  public static Command intakeCommand(Intake intake, Shooter shooter) {
    return Commands
        .either(
            Commands.none(),
            Commands.deadline(
                Commands
                    .sequence(
                        Commands.either(
                            new PivotToCmd<Intake>(
                                intake, IntakePivotPositions.RED_GROUND_INTAKE, true
                            ),
                            new PivotToCmd<Intake>(
                                intake, IntakePivotPositions.BLUE_GROUND_INTAKE, true
                            ),
                            Controlboard::isRedAlliance
                        ),
                        new PivotToCmd<Shooter>(shooter, ShooterPivotPositions.PASS_OFF, true)
                    )
                    .until(() -> { return shooter.isPieceInChute(); }),
                intake.getRoller().startEndRollerRPSWithFFCmd(IntakeConstants.ROLLER_IN_RPS),
                shooter.getRoller().startEndRollerRPSWithFFCmd(ShotConstants.ROLLER_IN_RPS),
                shooter.getRoller(ShooterConstants.FLYWHEEL_ID)
                    .startEndRollerRPSWithFFCmd(ShotConstants.FLYWHEEL_HOLD_RPS)
            ),
            shooter::isPieceInChute
        ) // TODO CHECk THIS RYAN URGENT
        .andThen(postIntakeCommand(intake, shooter));
  }

  public static Command postIntakeCommand(Intake intake, Shooter shooter) {
    Roller intakeRoller = intake.getRoller();
    Roller shooterRoller = shooter.getRoller();
    // add commands.sequence until seen again
    return Commands.either(
        Commands.sequence(
            shooterRoller.startEndRollerRPSWithFFCmd(ShotConstants.ROLLER_OUT_RPS).until(() -> {
              return !shooter.isPieceInChute();
            }),
            shooterRoller.startEndRollerRPSWithFFCmd(ShotConstants.ROLLER_SLOW_IN_RPS).until(() -> {
              return shooter.isPieceInChute();
            })
        ),
        Commands.sequence(
            Commands
                .parallel(
                    intakeRoller.startEndRollerRPSWithFFCmd(IntakeConstants.ROLLER_IN_RPS),
                    shooterRoller.startEndRollerRPSWithFFCmd(ShotConstants.ROLLER_IN_RPS)
                )
                .until(() -> { return shooter.isPieceInChute(); }),
            shooterRoller.startEndRollerRPSWithFFCmd(ShotConstants.ROLLER_OUT_RPS).until(() -> {
              return !shooter.isPieceInChute();
            }),
            shooterRoller.startEndRollerRPSWithFFCmd(ShotConstants.ROLLER_SLOW_IN_RPS).until(() -> {
              return shooter.isPieceInChute();
            })
        ),
        shooter::isPieceInChute
    );
  }

  public static Command prepAmpCommand(Intake intake, Shooter shooter, Leds leds) {
    return Commands.parallel(
        new SafePivotToCommand(
            intake, shooter, IntakePivotPositions.AMP, ShooterPivotPositions.AMP_PREP, true
        )
            .andThen(new PivotToCmd<Shooter>(shooter, ShooterPivotPositions.UNSAFE_AMP, true)),
        shooter.getRoller()
            .startEndRollerRPSWithFFCmd(ShotConstants.ROLLER_AMP_PREP_RPS)
            .until(() -> { return !shooter.isPieceInChute(); })
            .deadlineWith(new LedVariableSpeedCommand(
                leds, LedModes.WHITE_SOLID, shooter::getPercentOfSpeedLEDSpeedAmp
            ))
    );
  }

  public static Command scoreAmpCommand(Intake intake, Shooter shooter, Leds leds) {
    return Commands.sequence(
        new RollXCmd<Shooter>(shooter, ShooterRollerDistances.AMP_SHOT_DISTANCE),
        Commands.deadline(
            new PivotToCmd<Shooter>(
                shooter, ShooterPivotPositions.AMP, true, ShotConstants.AMP_PIVOT_VELO_RPS
            )
                .andThen(Commands.waitSeconds(ShotConstants.AMP_PRE_SHOT_WAIT_S)),
            Commands
                .waitUntil(
                    ()
                        -> AlgebraicUtils.cppMod(shooter.getPivot().getPivotPosition(), 1.0)
                        >= ShooterPivotPositions.AMP_UNTIL.getRotations()
                )
                .andThen(shooter.getRoller().startEndRollerRPSWithFFCmd(ShotConstants.ROLLER_AMP_RPS
                ))
        )
        // TODO: figure out stowing (drive back/turn??)
    );
  }

  public static Command passThroughGriefCommand(Intake intake, Shooter shooter) {
    return Commands.parallel(
        IntakeCommands.deployCommand(intake, shooter),
        intake.getRoller().startEndRollerRPSWithFFCmd(Intake.IntakeConstants.ROLLER_IN_RPS),
        shooter.getRoller().startEndRollerRPSWithFFCmd(ShotConstants.ROLLER_GRIEF_SPEED_RPS),
        shooter.getRoller().startEndRollerRPSWithFFCmd(ShotConstants.ROLLER_GRIEF_SPEED_RPS),
        shooter.getRoller(ShooterConstants.FLYWHEEL_ID)
            .startEndRollerRPSWithFFCmd(ShotConstants.FLYWHEEL_GRIEF_SPEED_RPS)
    );
  }
}
