package frc.robot.constants.enums;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.base.FaceAllianceTargetFFCommand;
import frc.robot.commands.complex.AimAllAtSpeakerCommand;
import frc.robot.commands.compound.AutonomousCommands;
import frc.robot.commands.compound.IntakeCommands;
import frc.robot.commands.compound.StaticScoreCommands;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.TagalongChoreoFollower;
import frc.robot.utils.TagalongSwerveModuleBase;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import javax.swing.Action;

// Mostly for PathPlanner, will need to be refactored for
public enum AutonomousActions {
  PREP_TO_SCORE_COMMAND(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return AutonomousCommands.prepToScoreCommand(drivetrain, shooter, intake);
      }
  ),
  SCORE_COMMAND(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return AutonomousCommands.scoreCommand(shooter);
      }
  ),
  PREP_FOR_AMP_COMMAND(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return AutonomousCommands.prepAmpCommand(intake, shooter, leds);
      }
  ),
  SHOT((Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
    return Commands
        .race(Commands.waitSeconds(1.0), new AimAllAtSpeakerCommand(drivetrain, shooter, intake))
        .finallyDo(() -> {
          {
            shooter.getPivot().setPivotProfile(ShooterPivotPositions.PASS_OFF);
            shooter.getPivot().setHoldPivotPosition(true);
          };
        });
  }),
  AMPSTART_SHOT_COMMAND(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return StaticScoreCommands
            .autonomousStaticShots(
                shooter, intake, ShooterPivotPositions.AUTONOMOUS_AMP_START_SPEAKER_SHOT
            )
            .withTimeout(1.0)
            .finallyDo(() -> {
              shooter.getPivot().setPivotProfile(ShooterPivotPositions.PASS_OFF);
              shooter.getPivot().setHoldPivotPosition(true);
              drivetrain.fieldRelativeDrive(0.0, 0.0, 0.0);
            })
            .beforeStarting(() -> TagalongChoreoFollower.autoAimDriver.initialize());
      }
  ),
  MIDSTART_SHOT_COMMAND(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return StaticScoreCommands
            .autonomousStaticShots(
                shooter, intake, ShooterPivotPositions.AUTONOMOUS_MID_START_SPEAKER_SHOT
            )
            .withTimeout(1.0)
            .finallyDo(() -> {
              shooter.getPivot().setPivotProfile(ShooterPivotPositions.PASS_OFF);
              shooter.getPivot().setHoldPivotPosition(true);
              drivetrain.fieldRelativeDrive(0.0, 0.0, 0.0);
            })
            .beforeStarting(() -> TagalongChoreoFollower.autoAimDriver.initialize());
      }
  ),
  SOURCESTART_SHOT_COMMAND(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return StaticScoreCommands
            .autonomousStaticShots(
                shooter, intake, ShooterPivotPositions.AUTONOMOUS_SOURCE_START_SPEAKER_SHOT
            )
            .withTimeout(1.0)
            .finallyDo(() -> {
              shooter.getPivot().setPivotProfile(ShooterPivotPositions.PASS_OFF);
              shooter.getPivot().setHoldPivotPosition(true);
              drivetrain.fieldRelativeDrive(0.0, 0.0, 0.0);
            })
            .beforeStarting(() -> TagalongChoreoFollower.autoAimDriver.initialize());
      }
  ),
  N1_SHOT_COMMAND(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return StaticScoreCommands
            .autonomousStaticShots(shooter, intake, ShooterPivotPositions.N1_SHOT)
            .deadlineWith(Commands
                              .run(() -> {
                                TagalongChoreoFollower.autoAimDriver.setChoreoAimDriverVelocities(
                                    0, 0
                                );
                                TagalongChoreoFollower.autoAimDriver.execute();
                              })
                              .asProxy())
            .finallyDo(() -> {
              shooter.getPivot().setPivotProfile(ShooterPivotPositions.PASS_OFF);
              shooter.getPivot().setHoldPivotPosition(true);
              drivetrain.fieldRelativeDrive(0.0, 0.0, 0.0);
            })
            .beforeStarting(() -> TagalongChoreoFollower.autoAimDriver.initialize());
      }
  ),
  N2_SHOT_COMMAND(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return StaticScoreCommands
            .autonomousStaticShots(shooter, intake, ShooterPivotPositions.N2_SHOT)
            .deadlineWith(Commands
                              .run(() -> {
                                TagalongChoreoFollower.autoAimDriver.setChoreoAimDriverVelocities(
                                    0, 0
                                );
                                TagalongChoreoFollower.autoAimDriver.execute();
                              })
                              .asProxy())
            .finallyDo(() -> {
              shooter.getPivot().setPivotProfile(ShooterPivotPositions.PASS_OFF);
              shooter.getPivot().setHoldPivotPosition(true);
              drivetrain.fieldRelativeDrive(0.0, 0.0, 0.0);
            })
            .beforeStarting(() -> TagalongChoreoFollower.autoAimDriver.initialize());
      }
  ),
  N3_SHOT_COMMAND(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return StaticScoreCommands
            .autonomousEjectStaticShots(shooter, intake, ShooterPivotPositions.N3_SHOT)
            .deadlineWith(Commands
                              .run(() -> {
                                TagalongChoreoFollower.autoAimDriver.setChoreoAimDriverVelocities(
                                    0, 0
                                );
                                TagalongChoreoFollower.autoAimDriver.execute();
                              })
                              .asProxy())
            .finallyDo(() -> {
              shooter.getPivot().setPivotProfile(ShooterPivotPositions.PASS_OFF);
              shooter.getPivot().setHoldPivotPosition(true);
              drivetrain.fieldRelativeDrive(0.0, 0.0, 0.0);
            })
            .beforeStarting(() -> TagalongChoreoFollower.autoAimDriver.initialize());
      }
  ),
  N4_SHOT_COMMAND(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return StaticScoreCommands
            .autonomousStaticShots(shooter, intake, ShooterPivotPositions.N4_SHOT)
            .deadlineWith(Commands
                              .run(() -> {
                                TagalongChoreoFollower.autoAimDriver.setChoreoAimDriverVelocities(
                                    0, 0
                                );
                                TagalongChoreoFollower.autoAimDriver.execute();
                              })
                              .asProxy())
            .finallyDo(() -> {
              shooter.getPivot().setPivotProfile(ShooterPivotPositions.PASS_OFF);
              shooter.getPivot().setHoldPivotPosition(true);
              drivetrain.fieldRelativeDrive(0.0, 0.0, 0.0);
            })
            .beforeStarting(() -> TagalongChoreoFollower.autoAimDriver.initialize());
      }
  ),
  N5_SHOT_COMMAND(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return StaticScoreCommands
            .autonomousStaticShots(shooter, intake, ShooterPivotPositions.N5_SHOT)
            .deadlineWith(Commands
                              .run(() -> {
                                TagalongChoreoFollower.autoAimDriver.setChoreoAimDriverVelocities(
                                    0, 0
                                );
                                TagalongChoreoFollower.autoAimDriver.execute();
                              })
                              .asProxy())
            .finallyDo(() -> {
              shooter.getPivot().setPivotProfile(ShooterPivotPositions.PASS_OFF);
              shooter.getPivot().setHoldPivotPosition(true);
              drivetrain.fieldRelativeDrive(0.0, 0.0, 0.0);
            })
            .beforeStarting(() -> TagalongChoreoFollower.autoAimDriver.initialize());
      }
  ),
  N6_SHOT_COMMAND(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return StaticScoreCommands
            .autonomousStaticShots(shooter, intake, ShooterPivotPositions.N6_SHOT)
            .deadlineWith(Commands
                              .run(() -> {
                                TagalongChoreoFollower.autoAimDriver.setChoreoAimDriverVelocities(
                                    0, 0
                                );
                                TagalongChoreoFollower.autoAimDriver.execute();
                              })
                              .asProxy())
            .finallyDo(() -> {
              shooter.getPivot().setPivotProfile(ShooterPivotPositions.PASS_OFF);
              shooter.getPivot().setHoldPivotPosition(true);
              drivetrain.fieldRelativeDrive(0.0, 0.0, 0.0);
            })
            .beforeStarting(() -> TagalongChoreoFollower.autoAimDriver.initialize());
      }
  ),
  SOURCE_FAR_SHOT_COMMAND(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return StaticScoreCommands
            .autonomousStaticShots(shooter, intake, ShooterPivotPositions.SOURCE_FAR_SHOT)
            .deadlineWith(Commands
                              .run(() -> {
                                TagalongChoreoFollower.autoAimDriver.setChoreoAimDriverVelocities(
                                    0, 0
                                );
                                TagalongChoreoFollower.autoAimDriver.execute();
                              })
                              .asProxy())
            .finallyDo(() -> {
              shooter.getPivot().setPivotProfile(ShooterPivotPositions.PASS_OFF);
              shooter.getPivot().setHoldPivotPosition(true);
              drivetrain.fieldRelativeDrive(0.0, 0.0, 0.0);
            })
            .beforeStarting(() -> TagalongChoreoFollower.autoAimDriver.initialize());
      }
  ),
  N7_SHOT_COMMAND(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return StaticScoreCommands
            .autonomousStaticShots(shooter, intake, ShooterPivotPositions.N7_SHOT)
            .deadlineWith(Commands
                              .run(() -> {
                                TagalongChoreoFollower.autoAimDriver.setChoreoAimDriverVelocities(
                                    0, 0
                                );
                                TagalongChoreoFollower.autoAimDriver.execute();
                              })
                              .asProxy())
            .finallyDo(() -> {
              shooter.getPivot().setPivotProfile(ShooterPivotPositions.PASS_OFF);
              shooter.getPivot().setHoldPivotPosition(true);
              drivetrain.fieldRelativeDrive(0.0, 0.0, 0.0);
            })
            .beforeStarting(() -> TagalongChoreoFollower.autoAimDriver.initialize());
      }
  ),
  AMP_FAR_SHOT_COMMAND(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return StaticScoreCommands
            .autonomousStaticShots(shooter, intake, ShooterPivotPositions.AMP_FAR_SHOT)
            .deadlineWith(Commands
                              .run(() -> {
                                TagalongChoreoFollower.autoAimDriver.setChoreoAimDriverVelocities(
                                    0, 0
                                );
                                TagalongChoreoFollower.autoAimDriver.execute();
                              })
                              .asProxy())
            .finallyDo(() -> {
              shooter.getPivot().setPivotProfile(ShooterPivotPositions.PASS_OFF);
              shooter.getPivot().setHoldPivotPosition(true);
              drivetrain.fieldRelativeDrive(0.0, 0.0, 0.0);
            })
            .beforeStarting(() -> TagalongChoreoFollower.autoAimDriver.initialize());
      }
  ),
  N8_SHOT_COMMAND(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return StaticScoreCommands
            .autonomousStaticShots(shooter, intake, ShooterPivotPositions.N8_SHOT)
            .deadlineWith(Commands
                              .run(() -> {
                                TagalongChoreoFollower.autoAimDriver.setChoreoAimDriverVelocities(
                                    0, 0
                                );
                                TagalongChoreoFollower.autoAimDriver.execute();
                              })
                              .asProxy())
            .finallyDo(() -> {
              shooter.getPivot().setPivotProfile(ShooterPivotPositions.PASS_OFF);
              shooter.getPivot().setHoldPivotPosition(true);
              drivetrain.fieldRelativeDrive(0.0, 0.0, 0.0);
            })
            .beforeStarting(() -> TagalongChoreoFollower.autoAimDriver.initialize());
      }
  ),
  AMP_COMMAND(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return AutonomousCommands.scoreAmpCommand(intake, shooter, leds);
      }
  ),
  POST_SCORE_COMMAND(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return AutonomousCommands.postScoreCommand(shooter);
      }
  ),
  PREP_TO_INTAKE_COMMAND(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return AutonomousCommands.prepToIntakeCommand(intake, shooter)
            .beforeStarting(() -> TagalongChoreoFollower.setAimingMode(false));
      }
  ),
  INTAKE_COMMAND(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return AutonomousCommands.intakeCommand(intake, shooter)
            .beforeStarting(() -> TagalongChoreoFollower.setAimingMode(false));
      }
  ),
  POST_INTAKE_COMMAND(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return AutonomousCommands.postIntakeCommand(intake, shooter)
            .beforeStarting(() -> TagalongChoreoFollower.setAimingMode(false));
      }
  ),
  PASS_THROUGH_GRIEF_COMMAND(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return AutonomousCommands.passThroughGriefCommand(intake, shooter);
      }
  ),
  STOW_COMMAND(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return IntakeCommands.stowCommand(intake, shooter);
      }
  ),
  CLEAR_OF_INTAKE(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return IntakeCommands.clearOfShooter(intake, shooter);
      }
  ),
  START_SHOOTING(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return new InstantCommand(
                   () -> { AimAllAtSpeakerCommand.startShooting(true); }
        ).beforeStarting(() -> TagalongChoreoFollower.setAimingMode(true));
      }
  ),
  FORCE_SHOT(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return new InstantCommand(() -> AimAllAtSpeakerCommand.forceShot());
      }
  ),
  STOP_SHOOTING(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return new InstantCommand(() -> AimAllAtSpeakerCommand.stopShooting());
      }
  ),
  STOP_AUTO_AIMING(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return new InstantCommand(() -> TagalongChoreoFollower.setAimingMode(false));
      }
  ),
  START_AUTO_AIMING(
      (Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds) -> {
        return new InstantCommand(() -> TagalongChoreoFollower.setAimingMode(true));
      }
  ),
  WAIT_COMMAND(0.5),
  WAIT_COMMAND4(4.0),
  PRINT("reached");

  @FunctionalInterface
  public interface Func<T1, T2, T3, T4, T5, R> {
    public R apply(T1 t1, T2 t2, T3 t3, T4 t4, T5 t5);
  }

  public static enum ActionTypes { BUILDABLE, OVERRIDE, PRINT, LED, WAIT }

  // All subsystems
  private static Drivetrain _drivetrain;
  private static Climber _climber;
  private static Intake _intake;
  private static Shooter _shooter;
  private static Leds _leds;

  public Command underlying;
  public Command command;
  private Func<Climber, Drivetrain, Intake, Shooter, Leds, Command> builder;
  private final ActionTypes actionType;

  private static final Map<AutonomousActions, Command> eventMap =
      new HashMap<AutonomousActions, Command>();

  AutonomousActions(Func<Climber, Drivetrain, Intake, Shooter, Leds, Command> builder) {
    actionType = ActionTypes.BUILDABLE;
    this.builder = builder;
  }

  // AutonomousActions(Command underlying) {
  //   this.underlying = underlying;
  //   actionType = ActionTypes.OVERRIDE;
  // }

  AutonomousActions(double waitSeconds) {
    this.underlying = Commands.waitSeconds(waitSeconds);
    actionType = ActionTypes.WAIT;
    this.command = this.underlying;
  }

  AutonomousActions(String message) {
    this.underlying = Commands.print(message);
    actionType = ActionTypes.PRINT;
    this.command = this.underlying;
  }

  // public static void registerAutonomousAction(String name, LedColors color) {
  //   AutonomousAction action = new AutonomousAction(
  //       name, new InstantCommand(() -> controller.setSolidColor(color, LedSections.ALL))
  //   );
  //   action.command = action.underlying;
  //   register(action);
  // }
  public static Map<AutonomousActions, Command> getEventMap() {
    return eventMap;
  }

  // All subsystems
  public static void registerAllAutonomousActions(
      Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds
  ) {
    _climber = climber;
    _drivetrain = drivetrain;
    _intake = intake;
    _shooter = shooter;
    _leds = leds;
    for (var action : AutonomousActions.values()) {
      if (action.actionType == ActionTypes.BUILDABLE) {
        action.underlying = action.builder.apply(climber, drivetrain, intake, shooter, leds);
        System.out.println("action = " + action);
        registerAutonomousAction(action);
      } else {
        registerNonBuildable(action);
      }
    }
  }

  public static void registerNonBuildable(AutonomousActions action) {
    eventMap.put(action, action.command);
  }

  public static void registerAutonomousAction(AutonomousActions action) {
    action.command = new InstantCommand(() -> action.underlying.schedule());
    eventMap.put(action, action.command);
  }

  public Command waitUntilUnderlyingFinished() {
    return Commands.waitUntil(() -> !underlying.isScheduled());
  }
}
