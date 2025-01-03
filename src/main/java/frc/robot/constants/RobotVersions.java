package frc.robot.constants;

import frc.robot.constants.confs.*;
import frc.robot.constants.confs.CompbotClimberConf;
import frc.robot.constants.confs.CompbotIntakeConf;
import frc.robot.constants.enums.CameraSets;
import frc.robot.parsers.json.ShooterConfJson;

public enum RobotVersions {
  COMP_BOT(
      "configs/drivetrain/compbotDrive.json",
      CompbotIntakeConf.construct(),
      CompbotShooterConf.construct(),
      "configs/shooter/compbotShooterConf.json",
      CompbotClimberConf.construct(),
      "configs/vision/cameras/compbotVision.json",
      CameraSets.THREE_CAMERAS,
      "configs/leds/compbotLeds.json"
  );
  // PRACTICE_BOT(
  // "configs/drivetrain/practicebotDrive.json",
  // null, // new PivotConf(),
  // null, // new RollerConf(),
  // null, // new TagalongBaseFlywheelConf(),
  // null, // new PivotConf(),
  // null, // new RollerConf(),
  // null, // new ElevatorConf(),
  // null, // new ElevatorConf(),
  // null, // new RollerConf(),
  // null,
  // // "configs/vision/cameras/practicebotVision.json",
  // CameraSets.NO_CAMERAS,
  // null),
  // SWERVE_BASE(
  // "configs/drivetrain/swervebaseDrive.json",
  // null, // new PivotConf(),
  // null, // new RollerConf(),
  // null, // new TagalongBaseFlywheelConf(),
  // null, // new PivotConf(),
  // null, // new RollerConf(),
  // null, // new ElevatorConf(),
  // null, // new ElevatorConf(),
  // null, // new RollerConf(),
  // null,
  // null,
  // null),
  // LEY("configs/drivetrain/leyDrive.json",
  // null, // new PivotConf(),
  // null, // new RollerConf(),
  // null, // new TagalongBaseFlywheelConf(),
  // null, // new PivotConf(),
  // null, // new RollerConf(),
  // null, // new ElevatorConf(),
  // null, // new ElevatorConf(),
  // null, // new RollerConf(),
  // null,
  // CameraSets.NO_CAMERAS,
  // null),
  // TEST_PIVOT(
  // null,
  // null, // new PivotConf(),
  // null, // new RollerConf(),
  // null, // new TagalongBaseFlywheelConf(),
  // null, // new PivotConf(),
  // null, // new RollerConf(),
  // null, // new ElevatorConf(),
  // null, // new ElevatorConf(),
  // null, // new RollerConf(),
  // null,
  // CameraSets.NO_CAMERAS,
  // null),
  // TEST_LEDS(
  // null,
  // null, // new PivotConf(),
  // null, // new RollerConf(),
  // null, // new TagalongBaseFlywheelConf(),
  // null, // new PivotConf(),
  // null, // new RollerConf(),
  // null, // new ElevatorConf(),
  // null, // new ElevatorConf(),
  // null, // new RollerConf(),
  // null,
  // CameraSets.NO_CAMERAS,
  // "configs/leds/testLeds.json");

  public final String _drive;
  public final BaseIntakeConf _intakeConf;
  public final BaseShooterConf _shooterConf;
  public final String _shooterParserFile;
  public final BaseClimberConf _climberConf;
  public final String _vision;
  public final CameraSets _cameraSets;
  public final String _leds;

  RobotVersions(
      String driveConfFile,
      BaseIntakeConf intakeConf,
      BaseShooterConf shooterConf,
      String shooterParserFile,
      BaseClimberConf climberConf,
      String visionConfFile,
      CameraSets camera,
      String ledFile
  ) {
    _drive = driveConfFile;
    _intakeConf = intakeConf;
    _shooterConf = shooterConf;
    _shooterParserFile = shooterParserFile;
    _climberConf = climberConf;
    _vision = visionConfFile;
    _cameraSets = camera;
    _leds = ledFile;
  }
}
