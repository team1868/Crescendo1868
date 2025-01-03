package frc.robot.commands.compound;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.base.ElevatorRaiseToCommand;
import frc.robot.commands.base.LedCommand;
import frc.robot.commands.base.LedVariableSpeedCommand;
import frc.robot.commands.base.PivotToCommand;
import frc.robot.commands.base.RollerRotateXCommand;
import frc.robot.commands.base.TeleopSwerveCommand;
import frc.robot.commands.complex.SafePivotToCommand;
import frc.robot.constants.enums.ClimberPositions;
import frc.robot.constants.enums.ClimberRollerDistances;
import frc.robot.constants.enums.DriveModes;
import frc.robot.constants.enums.IntakePivotPositions;
import frc.robot.constants.enums.LedColors;
import frc.robot.constants.enums.LedModes;
import frc.robot.constants.enums.ScoreTargets;
import frc.robot.constants.enums.ShooterPivotPositions;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ElevatorConstants;
import frc.robot.subsystems.Climber.RollerConstants;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShotConstants;

public class StaticScoreCommands {
  public static final Controlboard _controlboard = Controlboard.get();

  public static Command scoreSpeakerCommand(
      Drivetrain drivetrain,
      Intake intake,
      Shooter shooter,
      ShooterPivotPositions shotPivotPosition,
      Leds leds
  ) {
    return Commands
        .deadline(
            Commands.sequence(
                new SafePivotToCommand(
                    intake, shooter, IntakePivotPositions.CLEAR_OF_SHOOTER, shotPivotPosition, true
                ),
                Commands
                    .waitUntil(
                        ()
                            -> shotPivotPosition != ShooterPivotPositions.SPEAKER_SUBWOOFER_SHOT
                            ? shooter.isFlywheelAtSpeakerSpeed()
                            : shooter.getFlywheel().isFlywheelAtTargetSpeed(
                                ShotConstants.SUBWOOFER_SPEAKER_SPEED_RPS
                            )
                    )
                    .deadlineWith(new LedVariableSpeedCommand(
                        leds,
                        LedModes.VARIABLE_SPEED_BROWN_BLINKING,
                        shooter::getPercentOfSpeedLEDSpeedSpeaker
                    ))
                    .finallyDo(() -> { leds.unsafeSetLEDMode(LedModes.GREEN_BLINKING, 0); }),
                shoot(shooter, ShotConstants.ROLLER_SPEAKER_RPS)
                    .deadlineWith(new LedCommand(leds, LedModes.CYAN_BLINKING))
            ),
            shooter.getFlywheel().startEndFlywheelRPSWithFFCommand(
                shotPivotPosition != ShooterPivotPositions.SPEAKER_SUBWOOFER_SHOT
                    ? ShotConstants.FLYWHEEL_SPEAKER_SPEED_RPS
                    : ShotConstants.SUBWOOFER_SPEAKER_SPEED_RPS
            )
        )
        .andThen(IntakeCommands.partialStowCommand(intake, shooter))
        .beforeStarting(() -> { leds.unsafeSetLEDMode(LedModes.YELLOW_BLINKING, 0); })
        .finallyDo(() -> {
          leds.unsafeSetLEDMode(
              shooter.isPieceInChute() ? LedModes.GREEN_SOLID : LedModes.RED_SOLID, 0
          );
        });
  }

  public static Command scoreSpeakerCheckCommand(
      Drivetrain drivetrain,
      Intake intake,
      Shooter shooter,
      ShooterPivotPositions shotPivotPosition,
      Leds leds
  ) {
    return Commands.deadline(
        Commands.sequence(
            new SafePivotToCommand(
                intake, shooter, IntakePivotPositions.CLEAR_OF_SHOOTER, shotPivotPosition, true
            ),
            Commands
                .waitUntil(
                    ()
                        -> shooter.getFlywheel().isFlywheelAtTargetSpeed(
                            ShotConstants.FLYWHEEL_SPEAKER_CHECK_SPEED_RPS
                        )
                )
                .deadlineWith(new LedVariableSpeedCommand(
                    leds,
                    LedModes.VARIABLE_SPEED_BROWN_BLINKING,
                    shooter::getPercentOfSpeedLEDSpeedSpeaker
                )),
            shoot(shooter, ShotConstants.ROLLER_SPEAKER_RPS)
                .deadlineWith(new LedCommand(leds, LedModes.CYAN_BLINKING))
        ),
        shooter.getFlywheel().startEndFlywheelRPSWithFFCommand(
            ShotConstants.FLYWHEEL_SPEAKER_CHECK_SPEED_RPS
        )
    );
    // .andThen(IntakeCommands.partialStowCommand(intake, shooter));
  }

  public static Command scoreSpeakerCommand(
      Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds
  ) {
    return Commands
        .deadline(
            Commands.sequence(
                Commands.either(
                    new SafePivotToCommand(
                        intake,
                        shooter,
                        IntakePivotPositions.RED_GROUND_INTAKE.getRotations(),
                        shooter.getLaunchAngleFromTargetPhysicsRad(
                            drivetrain.getPose().getTranslation(),
                            ScoreTargets.SPEAKER_BOTTOM.getTranslation3d(Controlboard.isRedAlliance(
                            ))
                        ),
                        true
                    ),
                    new SafePivotToCommand(
                        intake,
                        shooter,
                        IntakePivotPositions.BLUE_GROUND_INTAKE.getRotations(),
                        shooter.getLaunchAngleFromTargetPhysicsRad(
                            drivetrain.getPose().getTranslation(),
                            ScoreTargets.SPEAKER_BOTTOM.getTranslation3d(Controlboard.isRedAlliance(
                            ))
                        ),
                        true
                    ),
                    Controlboard::isRedAlliance
                ),
                Commands.waitUntil(() -> shooter.isFlywheelAtSpeakerSpeed())
                    .deadlineWith(new LedVariableSpeedCommand(
                        leds,
                        LedModes.VARIABLE_SPEED_BROWN_BLINKING,
                        shooter::getPercentOfSpeedLEDSpeedSpeaker
                    )),
                shoot(shooter, ShotConstants.ROLLER_SPEAKER_RPS)
                    .deadlineWith(new LedCommand(leds, LedModes.CYAN_BLINKING))
            ),
            shooter.getFlywheel().startEndFlywheelRPSWithFFCommand(
                ShotConstants.FLYWHEEL_SPEAKER_SPEED_RPS
            )
        )
        .andThen(IntakeCommands.partialStowCommand(intake, shooter));
  }

  //   public static Command scoreAmpCommand(
  //       Drivetrain drivetrain, Intake intake, Shooter shooter, Controlboard controlboard, Leds
  //       leds
  //   ) {
  //     return Commands.sequence(
  //         Commands.deadline(
  //             new SafePivotToCommand(
  //                 intake, shooter, IntakePivotPositions.AMP, ShooterPivotPositions.AMP_PREP, true
  //             )
  //                 .andThen(new PivotToCommand(shooter, ShooterPivotPositions.UNSAFE_AMP, true))
  //                 .alongWith(shooter.getRoller()
  //                                .startEndRollerRPSWithFFCommand(ShotConstants.ROLLER_AMP_PREP_RPS)
  //                                .until(() -> { return !shooter.isPieceInChute(); })),
  //             new TeleopSwerveCommand(drivetrain, DriveModes.SNAP_TO_ANGLE)
  //                 .beforeStarting(() -> drivetrain.setSnapAngle(new Rotation2d(3.0 * Math.PI
  //                 / 2.0))),
  //             new LedVariableSpeedCommand(
  //                 leds, LedModes.WHITE_SOLID, shooter::getPercentOfSpeedLEDSpeedAmp
  //             )
  //         ),
  //         new TeleopSwerveCommand(drivetrain, DriveModes.SNAP_TO_ANGLE)
  //             .until(() -> !Controlboard.get().driverScoreAmpButton().getAsBoolean()),
  //         new RollerRotateXCommand<Shooter>(shooter, ShooterRollerDistances.AMP_SHOT_DISTANCE),
  //         new PivotToCommand(
  //             shooter, ShooterPivotPositions.AMP, true, ShotConstants.AMP_PIVOT_VELO_RPS
  //         ),
  //         //     .andThen(Commands.waitSeconds(ShotConstants.AMP_PRE_SHOT_WAIT_S)),
  //         // Commands
  //         //     .waitUntil(
  //         //         ()
  //         //             -> shooter.getPivot().getPivotAbsolutePositionRot()
  //         //             >= ShooterPivotPositions.AMP_UNTIL.getRotations()
  //         //     )
  //         //     .andThen(shooter.getRoller().startEndRollerRPSWithFFCommand(
  //         //         ShotConstants.ROLLER_AMP_RPS
  //         //     ))
  //         new PivotToCommand(shooter, ShooterPivotPositions.UNSAFE_AMP, true)
  //             .deadlineWith(shooter.getRoller().startEndRollerRPSCommand(ShotConstants.ROLLER_AMP_RPS
  //             )),
  //         IntakeCommands.partialStowCommand(intake, shooter)
  //             .deadlineWith(drivetrain
  //                               .run(() -> drivetrain.fieldRelativeAntiscrubDrive(0.0, -3.0,
  //                               0.0)) .withTimeout(ShotConstants.AMP_TIMEOUT_S))
  //         // TODO: figure out stowing (drive back/turn??)
  //     );
  //   }

  public static Command handOffCommand(Climber climber, Intake intake, Shooter shooter, Leds leds) {
    return Commands.sequence(
        // prep
        Commands.deadline(
            Commands.sequence(
                Commands.parallel(
                    new SafePivotToCommand(
                        intake,
                        shooter,
                        IntakePivotPositions.CLEAR_OF_SHOOTER,
                        ShooterPivotPositions.AMP2_INIT,
                        true
                    ),
                    new ElevatorRaiseToCommand<Climber>(climber, ClimberPositions.AMP2_INIT)
                ),
                // pass note
                Commands.deadline(
                    shooter.getRoller()
                        .startEndRollerRPSCommand(ShotConstants.AMP2_FLYWHEEL_RPS)
                        .until(() -> !shooter.isPieceInChute()),
                    climber.getRoller().startEndRollerRPSCommand(ShotConstants.AMP2_FLYWHEEL_RPS)
                )
            ),
            shooter.getFlywheel().startEndFlywheelRPSWithFFCommand(ShotConstants.AMP2_FLYWHEEL_RPS)
        ),
        new RollerRotateXCommand<Climber>(climber, ClimberRollerDistances.AMP2_PREP),
        Commands.parallel(
            new RollerRotateXCommand<Climber>(climber, ClimberRollerDistances.AMP2_PUSH),
            IntakeCommands.stowCommand(intake, shooter),
            new ElevatorRaiseToCommand<Climber>(
                climber, ClimberPositions.AMP2_INTERMEDIATE, ElevatorConstants.ELEVATOR_AMP_MPS
            )
        )
    );
  }

  public static Command scoreAmp2Command(
      Drivetrain drivetrain,
      Climber climber,
      Intake intake,
      Shooter shooter,
      Controlboard controlboard,
      Leds leds
  ) {
    return Commands
        .deadline(
            Commands.sequence(
                handOffCommand(climber, intake, shooter, leds),
                Commands.waitUntil(() -> !Controlboard.get().driverScoreAmpButton().getAsBoolean()),
                Commands.parallel(
                    climber.getRoller()
                        .startEndRollerRPSWithFFCommand(RollerConstants.ROLLER_AMP_SHOT)
                        .withTimeout(0.3),
                    new ElevatorRaiseToCommand<Climber>(
                        climber, ClimberPositions.AMP2_FINAL, ElevatorConstants.ELEVATOR_AMP_MPS
                    )
                )
            ),
            new TeleopSwerveCommand(drivetrain, DriveModes.SNAP_TO_ANGLE)
                .beforeStarting(() -> drivetrain.setSnapAngle(new Rotation2d(Math.PI / 2.0)))

        )
        .beforeStarting(() -> { leds.unsafeSetLEDMode(LedModes.SPOOKIES_BLUE_BLINKING, 0); })
        .finallyDo(() -> {
          leds.unsafeSetLEDMode(
              shooter.isPieceInChute() ? LedModes.GREEN_SOLID : LedModes.RED_SOLID, 0
          );
        });
  }

  private static Command prepForAmpScoreCommand(
      Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds
  ) {
    return Commands.parallel(
        new LedCommand(leds, LedModes.ORANGE_BLINKING)
        // Commands.sequence(
        //     shooter.startEndFlywheelRPSWithFFCommand(ShotConstants.FLYWHEEL_AMP_SPEED_RPS)
        //         .until(() -> shooter.isAtTargetSpeed(ShotConstants.FLYWHEEL_AMP_SPEED_RPS)),
        //     new RollerRotateXCommand<Shooter>(shooter, ShooterRollerDistances.AMP_PREP)
        // ), // TODO: use new rollers command version
        // TODO: replace with safe move
        // new SafePivotToCommand(
        //     intake, shooter, IntakePivotPositions.AMP_PREP, ShooterPivotPositions.AMP_PREP,
        //     true
        // )
    );
  }

  public static Command scoreTrapCommand(Intake intake, Shooter shooter) {
    return Commands.sequence(
        new SafePivotToCommand(
            intake, shooter, IntakePivotPositions.CLIMB_TRAP, ShooterPivotPositions.CLIMB_TRAP, true
        ),
        shoot(shooter, ShotConstants.ROLLER_TRAP_RPS),
        IntakeCommands.partialStowCommand(intake, shooter)
    );
  }
  private static Command shoot(Shooter shooter, double shotRPS, double timeout) {
    return Commands
        .deadline(
            Commands.waitUntil(() -> !shooter.isPieceInChute())
                .andThen(Commands.waitSeconds(ShotConstants.FLYWHEEL_WAIT_S)),
            shooter.getRoller().startEndRollerRPSWithFFCommand(shotRPS)
        )
        .withTimeout(timeout);
  }

  private static Command shoot(Shooter shooter, double shotRPS) {
    return shoot(shooter, shotRPS, ShotConstants.SHOT_TIMEOUT_S);
  }

  public static Command autonomousStaticShots(
      Shooter shooter, Intake intake, ShooterPivotPositions position
  ) {
    return Commands
        .deadline(
            Commands.sequence(
                new PivotToCommand(shooter, position, true)
                    .alongWith(AutonomousCommands.postIntakeCommand(intake, shooter))
                    .withTimeout(1.0),
                shoot(shooter, Shooter.ShotConstants.ROLLER_IN_RPS, 1.0)
            ),
            shooter.getFlywheel().startEndFlywheelRPSWithFFCommand(
                Shooter.ShotConstants.FLYWHEEL_SPEAKER_SPEED_RPS
            )
        )
        .finallyDo(() -> shooter.getPivot().setPivotProfile(ShooterPivotPositions.PASS_OFF));
  }

  public static Command autonomousEjectStaticShots(
      Shooter shooter, Intake intake, ShooterPivotPositions position
  ) {
    return Commands
        .either(
            Commands
                .deadline(
                    Commands.sequence(
                        new PivotToCommand<Shooter>(shooter, position, true)
                            .alongWith(AutonomousCommands.postIntakeCommand(intake, shooter))
                            .withTimeout(1.0),
                        shoot(shooter, Shooter.ShotConstants.ROLLER_IN_RPS, 1.0)
                    ),
                    shooter.getFlywheel().startEndFlywheelRPSWithFFCommand(
                        Shooter.ShotConstants.FLYWHEEL_SPEAKER_SPEED_RPS
                    )
                )
                .andThen(new PivotToCommand(shooter, ShooterPivotPositions.PASS_OFF, true)),
            intake.getRoller()
                .startEndRollerRPSWithFFCommand(IntakeConstants.ROLLER_EJECT_RPS)
                .withTimeout(0.3),
            () -> shooter.isPieceInChute()
        )
        .beforeStarting(Commands.race(
            Commands.waitSeconds(0.4), Commands.waitUntil(() -> shooter.isPieceInChute())
        ))
        .finallyDo(() -> {
          shooter.getPivot().setPivotProfile(ShooterPivotPositions.PASS_OFF);
          shooter.getPivot().setHoldPivotPosition(true);
        });
  }
}
