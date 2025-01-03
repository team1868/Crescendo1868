package frc.robot.commands.compound;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.base.LedCommand;
import frc.robot.commands.base.PivotToCommand;
import frc.robot.commands.base.RollerRotateXCommand;
import frc.robot.commands.base.TeleopSwerveCommand;
import frc.robot.commands.base.VisionPieceCommand;
import frc.robot.commands.complex.SafePivotToCommand;
import frc.robot.constants.Constants.Control;
import frc.robot.constants.enums.DriveModes;
import frc.robot.constants.enums.IntakePivotPositions;
import frc.robot.constants.enums.LedColors;
import frc.robot.constants.enums.LedModes;
import frc.robot.constants.enums.ShooterPivotPositions;
import frc.robot.constants.enums.ShooterRollerDistances;
import frc.robot.constants.enums.StaticTargets;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShotConstants;

public class IntakeCommands {
  public static Command groundIntakeCommand(Intake intake, Shooter shooter, Leds leds) {
    return Commands
        .sequence(
            Commands
                .parallel(
                    deployCommand(intake, shooter),
                    intake.getRoller().startEndRollerRPSWithFFCommand(IntakeConstants.ROLLER_IN_RPS
                    ),
                    shooter.getRoller().startEndRollerRPSWithFFCommand(ShotConstants.ROLLER_IN_RPS),
                    new LedCommand(leds, LedModes.PURPLE_SOLID)
                )
                .until(shooter::isPieceInChute),
            Commands.parallel(
                stowCommand(intake, shooter),
                shooter.getRoller()
                    .startEndRollerRPSWithFFCommand(-20)
                    .until(() -> !shooter.isPieceInChute())
                    .andThen(shooter.getRoller().startEndRollerRPSWithFFCommand(20).until(
                        shooter::isPieceInChute
                    ))
            )
        )
        .deadlineWith(shooter.getFlywheel().startEndFlywheelRPSWithFFCommand(-8))
        .beforeStarting(() -> { leds.unsafeSetLEDMode(LedModes.YELLOW_SOLID, 0); })
        .finallyDo(() -> {
          leds.unsafeSetLEDMode(
              shooter.isPieceInChute() ? LedModes.GREEN_SOLID : LedModes.RED_SOLID, 0
          );
        });
  }

  public static Command visionGroundIntakeCommand(
      Drivetrain drivetrain, Controlboard controlboard, Intake intake, Shooter shooter, Leds leds
  ) {
    return Commands.parallel(
        new VisionPieceCommand(drivetrain, controlboard), groundIntakeCommand(intake, shooter, leds)
    );
  }

  //   public static Command sourceIntakeShooterCommand(
  //       Drivetrain drivetrain, Intake intake, Shooter shooter
  //   ) {
  //     // Snap to angle until the chute has seen something
  //     // Stage 1 go to the  pivot intake positions rollers and flywheel on
  //     // Stage 2 wait until the chute has seen something
  //     // Stage 3 wait until the chute does not see, reverse teh rollers and turn off flywheel,
  //     teleop
  //     // swerve field relative no other constriant Stage 4 finish when the chute sees it again
  //     return Commands.deadline(
  //         Commands.sequence(
  //             Commands.deadline(
  //                 Commands.waitUntil(() -> shooter.isPieceInChute()),
  //                 new SafePivotToCommand(
  //                     intake,
  //                     shooter,
  //                     IntakePivotPositions.SOURCE_INTAKE,
  //                     ShooterPivotPositions.SOURCE_INTAKE,
  //                     true
  //                 ),
  //                 shooter.getFlywheel().setFlywheelRPSWithFFCommand(
  //                     ShotConstants.FLYWHEEL_SOURCE_SPEED_RPS
  //                 ),
  //                 shooter.getRoller().setRollerRPSWithFFCommand(ShotConstants.ROLLER_SOURCE_SPEED_RPS
  //                 )
  //             ),
  //             Commands.deadline(
  //                 Commands.waitUntil(() -> !shooter.isPieceInChute()),
  //                 shooter.getFlywheel().startEndFlywheelRPSWithFFCommand(
  //                     ShotConstants.FLYWHEEL_SOURCE_SPEED_RPS
  //                 ),
  //                 shooter.getRoller().startEndRollerRPSWithFFCommand(
  //                     ShotConstants.ROLLER_SOURCE_SPEED_RPS
  //                 )

  //                 // ,
  //                 // Commands.sequence(
  //                 //     new TeleopSwerveCommand(drivetrain, controlboard,
  //                 DriveModes.FIELD_RELATIVE)
  //                 //         .until(() ->
  //                 //         !Controlboard.get()._xboxDrive.rightBumper().getAsBoolean()),
  //                 //     drivetrain.forceFieldRelativeMovementCommand(
  //                 //         1.0 * Math.sin(Units.degreesToRadians(60.0)), 1.0, 2.0
  //                 //     )
  //                 // )
  //             ), // testing: rollers rotate cmd either rolls too much or not enough?
  //             new RollerRotateXCommand<Shooter>(
  //                 shooter, ShooterRollerDistances.MOVE_NOTE_FORWARD.getRotations()
  //             )
  //         ),
  //         new TeleopSwerveCommand(drivetrain, DriveModes.SNAP_TO_ANGLE)
  //             .beforeStarting(
  //                 ()
  //                     -> drivetrain.setSnapAngle(
  //                         StaticTargets.SOURCE.getPose2d(Controlboard.isRedAlliance()).getRotation()
  //                     )
  //             )
  //     );
  //   }

  /**
   * @param controlboard
   * @param drivetrain
   * @param intake
   * @param shooter
   * @return
   */
  public static Command sourceIntakeFeederCommand(
      Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds
  ) {
    // Snap to angle until the chute has seen something
    // Stage 1 go to the  pivot intake positions rollers and flywheel on
    // Stage 2 wait until the chute has seen something
    // Stage 3 wait until the chute does not see, reverse teh rollers and turn off flywheel, teleop
    // swerve field relative no other constriant Stage 4 finish when the chute sees it again
    return Commands
        .sequence(
            Commands.deadline(
                Commands.waitUntil(() -> shooter.isPieceInChute()),
                new SafePivotToCommand(
                    intake,
                    shooter,
                    IntakePivotPositions.SOURCE_FEEDER_INTAKE,
                    ShooterPivotPositions.SOURCE_FEEDER_INTAKE,
                    true
                ),
                shooter.getRoller().setRollerRPSWithFFCommand(
                    ShotConstants.ROLLER_SOURCE_SPEED_2_RPS
                ),
                new TeleopSwerveCommand(drivetrain, DriveModes.SNAP_TO_ANGLE)
                    .beforeStarting(
                        ()
                            -> drivetrain.setSnapAngle(StaticTargets.SOURCE
                                                           .getPose2d(Controlboard.isRedAlliance())
                                                           .getRotation())
                    )
            ),
            Commands.sequence(
                new TeleopSwerveCommand(drivetrain, DriveModes.ROBOT_CENTRIC)
                    .until(() -> !Controlboard.driverSourceIntakeButton().getAsBoolean()),
                drivetrain.forceRobotRelativeMovementCommand(2.0, 0.0, 0.33)
            ),
            Commands
                .parallel(
                    stowCommand(intake, shooter),
                    Commands.sequence(
                        shooter.getRoller()
                            .startEndRollerRPSWithFFCommand(ShotConstants.ROLLER_OUT_RPS)
                            .until(() -> { return !shooter.isPieceInChute(); }),
                        shooter.getRoller()
                            .startEndRollerRPSWithFFCommand(ShotConstants.ROLLER_SLOW_IN_RPS)
                            .until(() -> { return shooter.isPieceInChute(); })
                    )
                )
                .deadlineWith(
                    new TeleopSwerveCommand(drivetrain, DriveModes.FIELD_RELATIVE_ANTISCRUB)
                )
        )
        .beforeStarting(() -> { leds.unsafeSetLEDMode(LedModes.YELLOW_SOLID, 0); })
        .finallyDo(() -> {
          if (shooter.isPieceInChute()) {
            leds.unsafeSetLEDMode(LedModes.GREEN_SOLID, 0);
          } else if (shooter.getRoller().getRollerMotor().getVelocity().getValueAsDouble() <= 5.0) {
            leds.unsafeSetLEDMode(LedModes.ORANGE_SOLID, 0);
          } else {
            leds.unsafeSetLEDMode(LedModes.RED_SOLID, 0);
          }
        });
  }
  public static Command deployCommand(Intake intake, Shooter shooter) {
    return Commands.either(
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
    );
  }

  public static Command stowCommand(Intake intake, Shooter shooter) {
    return new SafePivotToCommand(
        intake, shooter, IntakePivotPositions.STOW, ShooterPivotPositions.STOW, true
    );
  }

  public static Command partialStowCommand(Intake intake, Shooter shooter) {
    return new SafePivotToCommand(
        intake, shooter, IntakePivotPositions.PARTIAL_STOW, ShooterPivotPositions.STOW, true
    );
  }

  public static Command clearOfShooter(Intake intake, Shooter shooter) {
    return new SafePivotToCommand(
        intake, shooter, IntakePivotPositions.CLEAR_OF_SHOOTER, ShooterPivotPositions.PASS_OFF, true
    );
  }

  public static Command transitionCommand(
      Intake intake, Shooter shooter

  ) {
    return transitionCommand(
        intake,
        shooter,
        shooter.getRoller()._defaultRollerLowerToleranceRot,
        shooter.getRoller()._defaultRollerUpperToleranceRot
    );
  }

  public static Command transitionCommand(
      Intake intake,
      Shooter shooter,
      double shooterRollerLowerTolerance,
      double shooterRollerUpperTolerance
  ) {
    return Commands.sequence(
        Commands
            .parallel(
                intake.getRoller().startEndRollerRPSWithFFCommand(
                    Intake.IntakeConstants.ROLLER_IN_RPS
                ),
                shooter.getRoller().startEndRollerRPSWithFFCommand(
                    Shooter.ShotConstants.ROLLER_IN_RPS
                )
            )
            .until(() -> shooter.isPieceInChute()),
        new RollerRotateXCommand<Shooter>(
            shooter,
            ShooterRollerDistances.MOVE_NOTE_BACK.getRotations(),
            shooterRollerLowerTolerance,
            shooterRollerUpperTolerance
        )
    );
  }

  public static Command clearToScoreCommand(Intake intake) {
    return new PivotToCommand<Intake>(intake, IntakePivotPositions.CLEAR_OF_SHOOTER, true);
  }
}
