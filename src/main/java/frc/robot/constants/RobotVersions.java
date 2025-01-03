package frc.robot.constants;

import frc.robot.constants.enums.CameraSets;

public enum RobotVersions {
  COMP_BOT(
      "configs/drivetrain/compbotDrive.json",
      "configs/intake/compbotIntakeConf.json",
      "configs/shooter/compbotShooterConf.json",
      "configs/climber/compbotClimberConf.json",
      "configs/vision/cameras/compbotVision.json",
      CameraSets.THREE_CAMERAS,
      "configs/leds/compbotLeds.json"
  ),
  PRACTICE_BOT(
      "configs/drivetrain/practicebotDrive.json",
      "configs/intake/practicebotIntakeConf.json",
      "configs/shooter/practicebotShooterConf.json",
      null,
      //   "configs/vision/cameras/practicebotVision.json",
      null,
      CameraSets.NO_CAMERAS,
      null
  ),
  SWERVE_BASE("configs/drivetrain/swervebaseDrive.json", null, null, null, null, null, null),
  LEY("configs/drivetrain/leyDrive.json",
      null,
      null,
      "configs/climber/climberConf.json",
      null,
      CameraSets.NO_CAMERAS,
      null),
  TEST_PIVOT(
      null, null, "configs/shooter/shooterConf.json", null, null, CameraSets.NO_CAMERAS, null
  ),
  TEST_LEDS(null, null, null, null, null, CameraSets.NO_CAMERAS, "configs/leds/testLeds.json");

  public final String _drive;
  public final String _intake;
  public final String _shooter;
  public final String _climber;
  public final String _vision;
  public final CameraSets _cameraSets;
  public final String _leds;

  RobotVersions(
      String driveConfFile,
      String intakeConfFile,
      String shooterConfFile,
      String climberConfFile,
      String visionConfFile,
      CameraSets camera,
      String ledFile
  ) {
    _drive = driveConfFile;
    _intake = intakeConfFile;
    _shooter = shooterConfFile;
    _climber = climberConfFile;
    _vision = visionConfFile;
    _cameraSets = camera;
    _leds = ledFile;
  }
}
