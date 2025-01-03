package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.base.AimShooterAtTarget;
import frc.robot.commands.base.FaceAllianceTargetFFCommand;
import frc.robot.commands.base.LedVariableSpeedCommand;
import frc.robot.commands.complex.SOTFCommand;
import frc.robot.commands.complex.SafePivotToCommand;
import frc.robot.constants.enums.IntakePivotPositions;
import frc.robot.constants.enums.LedColors;
import frc.robot.constants.enums.LedModes;
import frc.robot.constants.enums.ScoreTargets;
import frc.robot.constants.enums.ShooterPivotPositions;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShotConstants;
import tagalong.commands.base.PivotToCmd;

public class ScoreSpeakerCommands {
  public static Command ScoreSpeakerOnTheFlyCommand(
      Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds
  ) {
    return Commands
        .deadline(
            Commands.sequence(
                Commands.waitUntil(
                    () -> !Controlboard.driverScoreSpeakerOnFlyButton().getAsBoolean()
                ),
                Commands
                    .waitUntil(
                        ()
                            -> SOTFCommand._inTargetRange && shooter.isFlywheelAtSpeakerSpeed()
                            && SOTFCommand._onPivotTarget && SOTFCommand._onYawTarget
                    )
                    .deadlineWith(new LedVariableSpeedCommand(
                        leds,
                        LedModes.VARIABLE_SPEED_LIME_GREEN_BLINKING,
                        shooter::getPercentOfSpeedLEDSpeedSpeaker
                    ))
                    .finallyDo(() -> leds.unsafeSetLEDMode(LedModes.GREEN_BLINKING, 0)),
                Commands.deadline(
                    Commands.waitUntil(() -> !shooter.isPieceInChute())
                        .andThen(Commands.waitSeconds(ShotConstants.FLYWHEEL_WAIT_S)),
                    shooter.getRoller().startEndRollerRPSWithFFCmd(ShotConstants.ROLLER_SPEAKER_RPS)
                )
            ),
            Commands.sequence(
                new PivotToCmd<Intake>(
                    intake,
                    IntakePivotPositions.AUTO_AIM_CLEARANCE,
                    true,
                    intake.getPivot()._maxVelocityRPS,
                    0.02777777778,
                    0.02777777778
                )
                    .deadlineWith(new FaceAllianceTargetFFCommand(
                        drivetrain,
                        ScoreTargets.SPEAKER_BOTTOM,
                        ScoreTargets.SPEAKER_LEFT_BOTTOM,
                        ScoreTargets.SPEAKER_RIGHT_BOTTOM
                    )),
                new SOTFCommand(drivetrain, shooter, intake)
            ),
            shooter.getRoller(ShooterConstants.FLYWHEEL_ID)
                .startEndRollerRPSWithFFCmd(ShotConstants.FLYWHEEL_SPEAKER_SPEED_RPS)
        )
        .beforeStarting(() -> {
          drivetrain.setShootingVisionMode();
          leds.unsafeSetLEDMode(LedModes.YELLOW_BLINKING, 0);
        })
        .finallyDo(() -> {
          drivetrain.setTeleopVisionMode();
          leds.unsafeSetLEDMode(
              shooter.isPieceInChute() ? LedModes.GREEN_SOLID : LedModes.RED_SOLID, 0
          );
        });
  }

  public static Command SkipNoteCommand(
      Drivetrain drivetrain, Intake intake, Shooter shooter, ScoreTargets target, Leds leds
  ) {
    // Controlboard controlboard = Controlboard.get();
    return Commands
        .deadline(
            Commands.sequence(
                Commands.waitUntil(() -> !Controlboard.driverSkipNoteButton().getAsBoolean()),
                Commands.waitUntil(
                    (
                    ) -> shooter.isFlywheelAtSpeakerSpeed() && FaceAllianceTargetFFCommand._onTarget

                ),
                Commands.deadline(
                    Commands.waitUntil(() -> !shooter.isPieceInChute())
                        .andThen(Commands.waitSeconds(ShotConstants.FLYWHEEL_WAIT_S)),
                    shooter.getRoller().startEndRollerRPSWithFFCmd(ShotConstants.ROLLER_SPEAKER_RPS)
                )
            ),
            Commands.sequence(
                new SafePivotToCommand(
                    intake,
                    shooter,
                    IntakePivotPositions.CLEAR_OF_SHOOTER,
                    ShooterPivotPositions.SKIP_SHOT, // TODO
                                                     // change
                                                     // to
                                                     // current
                                                     // position
                    true
                ),
                new FaceAllianceTargetFFCommand(drivetrain, target)
            ),
            shooter.getRoller(ShooterConstants.FLYWHEEL_ID)
                .startEndRollerRPSWithFFCmd(ShotConstants.FLYWHEEL_SPEAKER_SPEED_RPS)
        )
        .beforeStarting(() -> { leds.unsafeSetLEDMode(LedModes.SPOOKIES_BLUE_LARSON, 0); })
        .finallyDo(() -> {
          leds.unsafeSetLEDMode(
              shooter.isPieceInChute() ? LedModes.GREEN_SOLID : LedModes.RED_SOLID, 0
          );
        });
  }

  private static Command AimAtTargetCommand(
      Drivetrain drivetrain, Shooter shooter, ScoreTargets target
  ) {
    return Commands.parallel(
        new FaceAllianceTargetFFCommand(drivetrain, target),
        new AimShooterAtTarget(drivetrain, shooter, target)
    );
  }
}
