// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.base.AimShooterAtTarget;
import frc.robot.commands.base.CrescendoTeleopLedCommand;
import frc.robot.commands.base.ElevatorPrepCommand;
import frc.robot.commands.base.FaceAllianceTargetCommand;
import frc.robot.commands.base.FaceAllianceTargetFFCommand;
import frc.robot.commands.base.GoToPoseCommand;
import frc.robot.commands.base.LedCommand;
import frc.robot.commands.base.PreMatchLedCommand;
import frc.robot.commands.base.TeleopSwerveCommand;
import frc.robot.commands.complex.AimAllAtSpeakerCommand;
import frc.robot.commands.complex.ResetAllCommand;
import frc.robot.commands.complex.SafePivotToCommand;
import frc.robot.commands.compound.AutonomousCommands;
import frc.robot.commands.compound.ClimbCommands;
import frc.robot.commands.compound.IntakeCommands;
import frc.robot.commands.compound.NoteCommands;
import frc.robot.commands.compound.ScoreSpeakerCommands;
import frc.robot.commands.compound.StaticScoreCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Control;
import frc.robot.constants.RobotAltModes;
import frc.robot.constants.enums.AutonomousActions;
import frc.robot.constants.enums.AutonomousRoutines;
import frc.robot.constants.enums.ClimberPositions;
import frc.robot.constants.enums.DriveModes;
import frc.robot.constants.enums.HookPositions;
import frc.robot.constants.enums.IntakePivotPositions;
import frc.robot.constants.enums.LedModes;
import frc.robot.constants.enums.ScoreTargets;
import frc.robot.constants.enums.ShooterPivotPositions;
import frc.robot.constants.enums.ShuffleboardStatus;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.utils.LoopTimer;
import frc.robot.utils.TagalongChoreoFollower;
import tagalong.commands.base.ElevateToCmd;
import tagalong.commands.base.PivotToCmd;
import tagalong.commands.base.RollXCmd;
import tagalong.math.AlgebraicUtils;
import tagalong.math.GeometricUtils;
import tagalong.math.WpilibUtils;

public class RobotContainer {
  /* --- Shared Resources --- */
  private boolean _activateDefaultStow = false;

  /* --- Subsystems --- */
  public static final Controlboard _controlboard = Controlboard.get();
  public Drivetrain _drivetrain = new Drivetrain(Constants.CRobot._drive, Controlboard._field);
  public Climber _climber = new Climber(Constants.CRobot._climberConf);
  public Intake _intake = new Intake(Constants.CRobot._intakeConf);
  public Shooter _shooter =
      new Shooter(Constants.CRobot._shooterConf, Constants.CRobot._shooterParserFile);
  public Leds _leds = new Leds(Constants.CRobot._leds, _shooter::isPieceInChute);

  public LedCommand _ledCommandA = new LedCommand(_leds, LedModes.FIRE);
  public LedCommand _ledCommandX = new LedCommand(_leds, LedModes.RAINBOW);
  public LedCommand _ledCommandY = new LedCommand(_leds, LedModes.RGB_FADE);
  public LedCommand _endGameLedCommand = new LedCommand(_leds, LedModes.RAINBOW);

  /* --- Commands --- */
  // Synchronization commands
  private Command _resetAllCommand =
      new ResetAllCommand(_drivetrain, _intake, _shooter, _climber).ignoringDisable(true);

  // LED Commands
  private PreMatchLedCommand _preMatchLedCommand =
      new PreMatchLedCommand(_leds, _climber, _shooter, _intake, _drivetrain, null);
  private CrescendoTeleopLedCommand _teleopLedCommand = new CrescendoTeleopLedCommand(
      _leds, LedModes.SPOOKIES_BLUE_BLINKING, LedModes.SPOOKIES_BLUE_SOLID
  );

  // Driver controller feedback
  private InstantCommand _xboxRumbleCommand = _controlboard.driverRumbleCommand();
  private InstantCommand _xboxResetRumbleCommand = _controlboard.driverResetRumbleCommand();

  // Drivetrain
  private TeleopSwerveCommand _teleopSwerveCommand =
      new TeleopSwerveCommand(_drivetrain, DriveModes.FIELD_RELATIVE_ANTISCRUB);
  private Command _drive0 =
      new InstantCommand(() -> _drivetrain.fieldRelativeAntiscrubDrive(0.0, 0, 0), _drivetrain);
  private Command _drive1 =
      new InstantCommand(() -> _drivetrain.fieldRelativeAntiscrubDrive(0.5, 0, 0), _drivetrain);
  private Command _driveOpp1 =
      new InstantCommand(() -> _drivetrain.fieldRelativeAntiscrubDrive(-0.5, 0, 0), _drivetrain);

  private FaceAllianceTargetCommand _faceTarget =
      new FaceAllianceTargetCommand(_drivetrain, ScoreTargets.SPEAKER_BOTTOM);
  private FaceAllianceTargetCommand _faceTargetFF =
      new FaceAllianceTargetFFCommand(_drivetrain, ScoreTargets.SPEAKER_BOTTOM);
  private InstantCommand _setPoseCommand = new InstantCommand(() -> {
    var x = ScoreTargets.SPEAKER_BOTTOM.getTranslation2d(Controlboard.isRedAlliance()).getX()
        + (Controlboard.isRedAlliance() ? -Units.inchesToMeters(50.5) : Units.inchesToMeters(50.5));
    var y = ScoreTargets.SPEAKER_BOTTOM.getTranslation2d(Controlboard.isRedAlliance()).getY();
    _drivetrain.setPose(
        new Pose2d(new Translation2d(x, y), new Rotation2d()),
        Controlboard.isRedAlliance() ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180)
    );
  });
  private InstantCommand _setPoseRedWingCommand = new InstantCommand(() -> {
    _drivetrain.setPose(
        new Pose2d(new Translation2d(436.55, 299), new Rotation2d()),
        Controlboard.isRedAlliance() ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180)
    );
  });

  private Command _onTheFlySpeaker =
      ScoreSpeakerCommands.ScoreSpeakerOnTheFlyCommand(_drivetrain, _intake, _shooter, _leds);

  private InstantCommand _activateFaceTarget =
      new InstantCommand(() -> _teleopSwerveCommand.activateFaceTarget(new Translation2d(0, 0)));
  private InstantCommand _zeroGyroCommand = _drivetrain.zeroGyroCommand();
  private InstantCommand _zeroPoseCommand = _drivetrain.zeroPoseCommand();
  private Command _forceChargingWheelDirection = _drivetrain.forceChargingWheelDirectionCommand();

  // Go to pose testing
  private GoToPoseCommand _goToPoseCommandPos1 =
      new GoToPoseCommand(_drivetrain, new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180.0)));
  private GoToPoseCommand _goToPoseCommandPos2 =
      new GoToPoseCommand(_drivetrain, new Pose2d(-2.0, -2.0, Rotation2d.fromDegrees(0.0)));

  /*
   * -------------------------------- BRINGUP COMMANDS
   * --------------------------------
   */
  // // /* -------- SHOOTER PIVOT BRINGUP -------- */
  // private PivotToCmd<Shooter> _shooterStow =
  // new PivotToCmd(_shooter, ShooterPivotPositions.STOW, true);
  // private PivotToCmd<Shooter> _shooterAmp =
  // new PivotToCmd(_shooter, ShooterPivotPositions.AMP, true);
  // private PivotToCmd<Shooter> _shooterZero =
  // new PivotToCmd(_shooter, ShooterPivotPositions.ZERO, true);
  // private PivotToCmd<Shooter> _shooterGround =
  // new PivotToCmd(_shooter, ShooterPivotPositions.PASS_OFF, true);
  // private PivotToCmd<Shooter> _shooterClear =
  // new PivotToCmd(_shooter, ShooterPivotPositions.AMP_PREP, true);

  // /* -------- INTAKE PIVOT BRINGUP -------- */
  // private PivotToCmd<Intake> _intakeZero = new PivotToCmd(_intake,
  // IntakePivotPositions.ZERO, true); private PivotToCmd<Intake>
  // _intakeGround =
  // new PivotToCmd(_intake, Controlboard.isRedAlliance() ?
  // IntakePivotPositions.RED_GROUND_INTAKE :
  // IntakePivotPositions.BLUE_GROUND_INTAKE, true);
  // private PivotToCmd<Intake> _intakeStow = new PivotToCmd(_intake,
  // IntakePivotPositions.STOW, true); private PivotToCmd<Intake> _intakeClear
  // =
  // new PivotToCmd(_intake, IntakePivotPositions.CLEAR_OF_SHOOTER, true);

  // /* -------- SAFE PIVOT BRINGUP -------- */
  // // ZONE 0
  // private SafePivotToCommand _zone0 = new SafePivotToCommand(
  // _intake, _shooter, IntakePivotPositions.CLEAR_OF_SHOOTER,
  // ShooterPivotPositions.AMP_PREP,
  // true
  // );
  // // ZONE 1
  // private SafePivotToCommand _zone1 = new SafePivotToCommand(
  // _intake, _shooter, IntakePivotPositions.CLEAR_OF_SHOOTER,
  // ShooterPivotPositions.AMP_PREP,
  // true
  // );
  // // ZONE 2 -- _zero for both mechanisms
  // private SafePivotToCommand _zero = new SafePivotToCommand(
  // _intake, _shooter, IntakePivotPositions.ZERO, ShooterPivotPositions.ZERO,
  // true
  // );
  // // NO ZONE 3 DANGER DANGER
  // // ZONE 4
  // private SafePivotToCommand<Shooter> _shooterTest2 = new SafePivotToCommand(
  // _intake, _shooter, Controlboard.isRedAlliance() ?
  // IntakePivotPositions.RED_GROUND_INTAKE :
  // IntakePivotPositions.BLUE_GROUND_INTAKE, ShooterPivotPositions.PASS_OFF, true
  // );
  // // ZONE 5
  // private SafePivotToCommand _zone5 = new SafePivotToCommand(
  // _intake, _shooter, IntakePivotPositions.STOW, ShooterPivotPositions.STOW,
  // true
  // );

  private AimShooterAtTarget _aimShooterAtTargetCommand =
      new AimShooterAtTarget(_drivetrain, _shooter, ScoreTargets.SPEAKER_BOTTOM);
  // private PivotToCmd<Intake> _intakeGroundPivot =
  // new PivotToCmd(_intake, Controlboard.isRedAlliance() ?
  // IntakePivotPositions.RED_GROUND_INTAKE :
  // IntakePivotPositions.BLUE_GROUND_INTAKE, true);
  // private PivotToCmd<Intake> _intakeStowPivot =
  // new PivotToCmd(_intake, IntakePivotPositions.STOW, true);
  // private PivotToCmd<Intake> _intakePartialStowPivot =
  // new PivotToCmd(_intake, IntakePivotPositions.PARTIAL_STOW, true);

  private Command _shooterRollerFF = Commands.startEnd(() -> {
    _shooter.getRoller().setRollerVelocity(90, true);
    System.out.println(
        "\t curPivotAngleDeg "
        + (360.0
           - Units.rotationsToDegrees(
               AlgebraicUtils.cppMod(_shooter.getPivot().getPivotPosition(), 1.0)
           ))
        + "\t cur flywheel vel"
        + _shooter.getRoller(ShooterConstants.FLYWHEEL_ID).getPrimaryMotor().getVelocity()
    );
  }, () -> _shooter.getRoller().setRollerPower(0.0));

  private Command _intakeRollerFF = _intake.getRoller().startEndRollerRPSWithFFCmd(90);
  private Command _flywheelRollerFF = _shooter.getRoller().startEndRollerRPSWithFFCmd(-70);

  private Command _flywheelWhileOnCommand =
      _shooter.getRoller(ShooterConstants.FLYWHEEL_ID).setRollerPowerCmd(0.5);
  private Command _setFlywheelRPSCommand =
      _shooter.getRoller(ShooterConstants.FLYWHEEL_ID).setRollerRPSCmd(50);
  private Command _setFlywheelRPSwFFCommand =
      _shooter.getRoller(ShooterConstants.FLYWHEEL_ID).setRollerRPSWithFFCmd(50);

  // private RollXCmd<Shooter> _shooterRotateXCommand =
  // new RollXCmd<Shooter> (_shooter, 0.25, 0.1, 0.1);
  // private RollXCmd<Shooter> _shooterRotateXCommand = new
  // RollXCmd<Shooter>(_shooter, 20.0, true, 0.1,
  // 0.1);
  // private Command _feedUntilLimit =
  // _shooter.getRoller().startEndRollerRPSWithFFCmd(50.0).until(
  // () -> _shooter.isPieceInChute());
  // private RollXCmd<Intake> _intakeRotateXCommand = new RollXCmd<Intake>(
  // _intake,
  // 10.0,
  // true);

  // Climber
  private ElevateToCmd _climberRaiseToCommand =
      new ElevateToCmd(_climber, ClimberPositions.AMP2_INIT, true);
  private ElevateToCmd _climbRaiseToTrap =
      new ElevateToCmd(_climber, ClimberPositions.TRAP_POSITION, true);

  // Autonomous testing
  private Command _prepToScore =
      AutonomousCommands.prepToScoreCommand(_drivetrain, _shooter, _intake);
  private Command _score = AutonomousCommands.scoreCommand(_shooter);
  private Command _postScore = AutonomousCommands.postScoreCommand(_shooter);
  private Command _prepToIntake = AutonomousCommands.prepToIntakeCommand(_intake, _shooter);
  private Command _intakeCommand = AutonomousCommands.intakeCommand(_intake, _shooter);
  private Command _postIntake = AutonomousCommands.postIntakeCommand(_intake, _shooter);
  private Command _greifPass = AutonomousCommands.passThroughGriefCommand(_intake, _shooter);
  private Command _prepAmp = AutonomousCommands.prepAmpCommand(_intake, _shooter, _leds);
  private Command _ampScore = AutonomousCommands.scoreAmpCommand(_intake, _shooter, _leds);
  // private Command _ampScore = AutonomousCommands.scoreAmpCommand();

  // * -------- Final Commands -------- */
  private Command _groundIntakeCommand =
      IntakeCommands.groundIntakeCommand(_intake, _shooter, _leds);
  private Command _ejectCommand = NoteCommands.ejectNoteCommand(_intake, _shooter, _leds);
  private Command _visionGroundIntakeCommand = IntakeCommands.visionGroundIntakeCommand(
      _drivetrain, _controlboard, _intake, _shooter, _leds
  );

  private Command _scoreSpeakerCommand =
      StaticScoreCommands.scoreSpeakerCommand(_drivetrain, _intake, _shooter, _leds);
  // private ScoreSpeakerOnTheFlyCommand2 _scoreSpeakerCommand2 =
  // new ScoreSpeakerOnTheFlyCommand2(_drivetrain, _shooter);
  // private ScoreSpeakerOnTheFlyCommand3 _scoreSpeakerCommand3 =
  // new ScoreSpeakerOnTheFlyCommand3(_drivetrain, _shooter);
  // private ScoreSpeakerOnTheFlyCommand4 _scoreSpeakerCommand4 =
  // new ScoreSpeakerOnTheFlyCommand4(_drivetrain, _shooter);
  private Command _skipCommand = ScoreSpeakerCommands.SkipNoteCommand(
      _drivetrain, _intake, _shooter, ScoreTargets.AMP_CORNER, _leds
  );
  private Command _launchAtAmpCommand =
      NoteCommands.launchAtAmpCommand(_drivetrain, _shooter, _intake, _leds);

  // private Command _scoreAmpCommand =
  // StaticScoreCommands.scoreAmpCommand(_drivetrain, _intake, _shooter,
  // _controlboard, _leds);
  private Command _scoreAmp2Command = StaticScoreCommands.scoreAmp2Command(
      _drivetrain, _climber, _intake, _shooter, _controlboard, _leds
  );

  private Command _scoreSubwooferCommand = StaticScoreCommands.scoreSpeakerCommand(
      _drivetrain, _intake, _shooter, ShooterPivotPositions.SPEAKER_SUBWOOFER_SHOT, _leds
  );

  private Command _stowCommand = IntakeCommands.stowCommand(_intake, _shooter);
  private Command _partialStowCommand = IntakeCommands.partialStowCommand(_intake, _shooter);

  /* -------- Shuffleboard -------- */
  private Command _pivotGroundIntakeCommand = IntakeCommands.deployCommand(_intake, _shooter);
  private SafePivotToCommand _pivotAmpComand = new SafePivotToCommand(
      _intake,
      _shooter,
      IntakePivotPositions.CLEAR_OF_SHOOTER,
      ShooterPivotPositions.AMP2_INIT,
      true
  );
  private SafePivotToCommand _pivotSpeakerSubwooferCommand = new SafePivotToCommand(
      _intake,
      _shooter,
      IntakePivotPositions.CLEAR_OF_SHOOTER,
      ShooterPivotPositions.SPEAKER_SUBWOOFER_SHOT,
      true
  );
  private SafePivotToCommand _pivotZeroCommand = new SafePivotToCommand(
      _intake, _shooter, IntakePivotPositions.ZERO, ShooterPivotPositions.ZERO, true
  );
  private Command _scoreSubwooferCheckCommand = StaticScoreCommands.scoreSpeakerCheckCommand(
      _drivetrain, _intake, _shooter, ShooterPivotPositions.SPEAKER_SUBWOOFER_SHOT, _leds
  );

  private ElevateToCmd<Climber> _climberStowCommand =
      new ElevateToCmd<Climber>(_climber, ClimberPositions.STOW, true);
  private ElevateToCmd<Climber> _climberExtendCommand =
      new ElevateToCmd<Climber>(_climber, ClimberPositions.EXTEND, true);
  private Command _raiseHooksClimbCommand = new ElevateToCmd<Climber>(
      Climber.ElevatorConstants.HOOKS_ID, _climber, ClimberPositions.CLIMB_EXTEND
  );
  private Command _lowerHooksClimbCommand = new ElevateToCmd<Climber>(
      Climber.ElevatorConstants.HOOKS_ID, _climber, ClimberPositions.CLIMB_RETRACT
  );
  private Command _elevatorSysCheckRezero =
      new ElevatorPrepCommand<Climber>(Climber.ElevatorConstants.HOOKS_ID, _climber)
          .withTimeout(0.25);
  // private Command _sourceShooterIntake =
  // IntakeCommands.sourceIntakeShooterCommand(_drivetrain, _intake, _shooter);
  private Command _sourceFeederIntake =
      IntakeCommands.sourceIntakeFeederCommand(_drivetrain, _intake, _shooter, _leds);

  // private Command _hookStowingCommand =
  // new ElevateToCmd(Climber.ElevatorConstants.HOOKS_ID, _climber,
  // HookPositions.ZERO)
  // .beforeStarting(
  // ()
  // -> _climber.getElevator(Climber.ElevatorConstants.HOOKS_ID)
  // .getPrimaryMotor()
  // .setPosition(0.082)
  // );

  // private Command _hookRaisingCommand =
  // new ElevateToCmd(Climber.ElevatorConstants.HOOKS_ID, _climber,
  // HookPositions.ZERO)
  // .beforeStarting(
  // ()
  // -> _climber.getElevator(Climber.ElevatorConstants.HOOKS_ID)
  // .getPrimaryMotor()
  // .setPosition(-0.082)
  // );

  // Auto and Path Planner
  private Alliance _alliance = Alliance.Blue;
  private SendableChooser<Integer> _autoChooser = new SendableChooser<Integer>();

  public AutonomousRoutines _curAutoSelected = AutonomousRoutines.DEFAULT_AUTO;

  private AutonomousRoutines[] _autonModes;

  // Verbose mode
  private double[] _curPoses = new double[4];
  private double[] _visionPoses = new double[4];

  private DataLog _log = DataLogManager.getLog();
  private DoubleArrayLogEntry _poseLog = new DoubleArrayLogEntry(_log, "/Swerve/RobotPose");
  private DoubleArrayLogEntry _visionPoseLog = new DoubleArrayLogEntry(_log, "/Swerve/VisionPose");
  private DoubleArrayLogEntry Log = new DoubleArrayLogEntry(_log, "/Swerve/Gyro");
  private DoubleArrayLogEntry _swerveSetpoints = new DoubleArrayLogEntry(_log, "/Swerve/Setpoints");
  private DoubleArrayLogEntry _swerveOutputs = new DoubleArrayLogEntry(_log, "/Swerve/RealOutputs");

  private DoubleLogEntry _posXLog = new DoubleLogEntry(_log, "/Drive/X");
  private DoubleLogEntry _posYLog = new DoubleLogEntry(_log, "/Drive/Y");
  private DoubleLogEntry _posThetaLog = new DoubleLogEntry(_log, "/Drive/Theta");

  public RobotContainer() {
    var setupTab = Shuffleboard.getTab("Setup");

    // Auto Chooser
    setupTab.add("Auto Chooser", _autoChooser).withSize(3, 2);
    _autoChooser.setDefaultOption("NO AUTONOMOUS", AutonomousRoutines.DEFAULT_AUTO.ordinal());

    // Default commands
    _drivetrain.setDefaultCommand(_teleopSwerveCommand);
    // _leds.setDefaultCommand(_teleopLedCommand);

    TagalongChoreoFollower.autoAimDriver =
        new AimAllAtSpeakerCommand(_drivetrain, _shooter, _intake);

    registerAutonomousRoutines(_climber, _drivetrain, _intake, _shooter, _leds);

    configShuffleboard();

    configureBindings();
    // configureTestBindings();

    LoopTimer.markCompletion("\n Robot Initialized: ");
  }

  private void registerAutonomousRoutines(
      Climber climber, Drivetrain drivetrain, Intake intake, Shooter shooter, Leds leds
  ) {
    if (_autonModes == null) {
      AutonomousActions.registerAllAutonomousActions(climber, drivetrain, intake, shooter, leds);
      _autonModes = AutonomousRoutines.values();
      System.out.println("auton modes " + _autonModes);
      // System.out.println("event map \n" + AutonomousActions.getEventMap());

      for (AutonomousRoutines routine : _autonModes) {
        if (routine.showInDashboard == ShuffleboardStatus.SHOW) {
          routine.build(drivetrain, shooter, intake, leds);
          _autoChooser.addOption(routine.shuffleboardName, routine.ordinal());
        }
      }
    }
  }

  public AutonomousRoutines getAutonomousRoutineSelection() {
    var result = _autoChooser.getSelected();
    return _autonModes[result == null ? 0 : result.intValue()];
  }

  public void configFMSData() {
    _alliance = DriverStation.getAlliance().orElse(_alliance);
    _controlboard.updateAlliance(_alliance);
  }

  protected void configShuffleboard() {
    /* Put commands for pit testing here */
    ShuffleboardTab systemCheckTab = Shuffleboard.getTab("Systems Check");

    // Shooter and Intake
    systemCheckTab.add("Pivot Stow", _stowCommand).withPosition(0, 0).withSize(2, 1);
    systemCheckTab.add("Pivot Zero", _pivotZeroCommand).withPosition(0, 1).withSize(2, 1);
    systemCheckTab.add("Pivot Ground ", _pivotGroundIntakeCommand)
        .withPosition(0, 2)
        .withSize(2, 1);
    systemCheckTab.add("Pivot Amp", _pivotAmpComand).withPosition(0, 3).withSize(2, 1);
    systemCheckTab.add("Pivot SubSpeaker", _pivotSpeakerSubwooferCommand)
        .withPosition(0, 4)
        .withSize(2, 1);

    // Climber
    systemCheckTab.add("Elevator Stow", _climberStowCommand).withPosition(2, 0).withSize(2, 1);
    systemCheckTab.add("Elevator Extend ", _climberExtendCommand).withPosition(3, 1).withSize(2, 1);
    systemCheckTab.add("Hook Move Down", _lowerHooksClimbCommand).withPosition(4, 0).withSize(2, 1);
    systemCheckTab.add("Hook Move Up ", _raiseHooksClimbCommand).withPosition(5, 1).withSize(2, 1);
    systemCheckTab.add("Hook Rezero and Retract", _elevatorSysCheckRezero)
        .withPosition(2, 4)
        .withSize(2, 2);

    // Drivetrain
    // systemCheckTab.add("Set Pose ", _setPoseCommand).withPosition(6,
    // 3).withSize(2, 1);

    // Controller actions
    systemCheckTab.add("Ground Intake ", _groundIntakeCommand).withPosition(7, 0).withSize(2, 1);
    systemCheckTab.add("Source Intake", _sourceFeederIntake).withPosition(7, 1).withSize(2, 1);
    systemCheckTab.add("Manual Shoot", _scoreSubwooferCheckCommand)
        .withPosition(9, 0)
        .withSize(2, 1);
    systemCheckTab.add("Score Amp", _scoreAmp2Command).withPosition(11, 0).withSize(2, 1);
    systemCheckTab.add("Launch Amp", _launchAtAmpCommand).withPosition(11, 1).withSize(2, 1);
    // systemCheckTab.add("Score Fly ", _onTheFlySpeaker).withPosition(9,
    // 1).withSize(2, 1);
  }

  public void robotInit() {
    configFMSData();
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.logNetworkTables(false);
    if (RobotAltModes.isVerboseMode) {
      DriverStation.startDataLog(_log);
    }

    // _preMatchLedCommand.schedule();
  }

  public void autonomousInit() {
    _drivetrain.setAutoVisionMode();
  }

  public void teleopInit() {
    _teleopSwerveCommand.teleopInit();
    _drivetrain.setTeleopVisionMode();
  }

  public void simulationInit() {
    _climber.simulationInit();
    _drivetrain.simulationInit();
    _intake.simulationInit();
    _shooter.simulationInit();
  }

  public void simulationPeriodic() {
    _climber.simulationPeriodic();
    _drivetrain.simulationPeriodic();
    _intake.simulationPeriodic();
    _shooter.simulationPeriodic();
  }

  public void log() {
    if (RobotAltModes.isVerboseMode) {
      Pose2d curPose = _drivetrain.getPose();
      WpilibUtils.poseToArray(curPose, _curPoses);
      _poseLog.append(_curPoses);

      _posXLog.append(curPose.getX());
      _posYLog.append(curPose.getY());
      _posThetaLog.append(curPose.getRotation().getDegrees());

      // GeometricUtils.poseToArray(_drivetrain.getVisionPose(), _visionPoses);
      // _visionPoseLog.append(_visionPoses);

      // Module Config:
      _swerveOutputs.append(_drivetrain.swerveMeasuredIO());
      _swerveSetpoints.append(_drivetrain.swerveSetpointsIO());
    }
  }

  public void periodic() {
    // controlboard is not a subsystem so the periodic function must be explicitly
    // called
    _controlboard.periodic();
  }

  public void disabledPeriodic() {
    AutonomousRoutines prev = _curAutoSelected;
    _curAutoSelected = getAutonomousRoutineSelection();

    Alliance prevAlliance = _alliance;
    _alliance = DriverStation.getAlliance().isEmpty() ? DriverStation.Alliance.Blue
                                                      : DriverStation.getAlliance().get();

    // For initial pose aligning
    if (prev != _curAutoSelected || prevAlliance != _alliance) {
      _controlboard.updateAlliance(_alliance);

      // Pose lighting here
    }

    // Disabled periodic for subsystem
    _drivetrain.disabledPeriodic();
  }

  public void onEnable() {
    _resetAllCommand.cancel();
    // _preMatchLedCommand.cancel();

    _intake.onEnable();
    _shooter.onEnable();
    _climber.onEnable();
    _drivetrain.onEnable();
  }

  public void onDisable() {
    CommandScheduler.getInstance().cancelAll();
    var driveCommand = _drivetrain.getCurrentCommand();
    if (driveCommand != null)
      driveCommand.cancel();
    _resetAllCommand.schedule();
    AimAllAtSpeakerCommand.stopShooting();

    _controlboard.driverResetRumble();
    _intake.onDisable();
    _shooter.onDisable();
    _climber.onDisable();
    _drivetrain.setPrematchVisionMode();
    _drivetrain.onDisable();
    _activateDefaultStow = false;
    // _preMatchLedCommand.schedule();
  }

  public void teleopPeriodic() {
    if (Timer.getMatchTime() <= 25 && !_endGameLedCommand.isScheduled()) {
      _endGameLedCommand.schedule();
    }

    if (_shooter.getCurrentCommand() == null && _intake.getCurrentCommand() == null) {
      if (_activateDefaultStow) {
        _partialStowCommand.schedule();
        _climberStowCommand.schedule();
      }
    } else {
      _activateDefaultStow = true;
    }
  }

  private void configureBindings() {
    System.out.println("COMP BUTTON BINDINGS");

    Controlboard.driverZeroGyroButton().onTrue(_zeroGyroCommand);
    Controlboard.driverSourceIntakeButton().toggleOnTrue(_sourceFeederIntake);

    Controlboard.driverGroundIntakeButton().toggleOnTrue(_groundIntakeCommand);
    Controlboard.driverEjectButton().toggleOnTrue(_ejectCommand);
    Controlboard.driverScoreAmpButton().onTrue(_scoreAmp2Command);

    // Controlboard._xboxDrive.rightTrigger().toggleOnTrue(_scoreSpeakerCommand);
    // Controlboard._xboxDrive.b().onTrue(_climb);
    // Controlboard._xboxDrive.x().whileTrue(_staticScoreSpeakerCommand);
    Controlboard._xboxDrive.a().toggleOnTrue(_visionGroundIntakeCommand);
    // Controlboard._xboxDrive.y().onTrue(_faceTargetFF);

    // Controlboard._xboxDrive.b().onTrue(_setPoseCommand);
    // Controlboard._xboxDrive.y().onTrue(_setPoseRedWingCommand);

    // for now
    Controlboard.driverScoreSpeakerSubwooferButton().whileTrue(_scoreSubwooferCommand);
    Controlboard.driverScoreSpeakerOnFlyButton().onTrue(_onTheFlySpeaker);
    // Controlboard.driverSkipNoteButton().toggleOnTrue(_skipCommand);
    Controlboard.driverLaunchAtAmpButton().onTrue(_launchAtAmpCommand);

    Controlboard.driverClimbExtendButton().onTrue(_raiseHooksClimbCommand);
    Controlboard.driverClimbRetractButton().onTrue(_lowerHooksClimbCommand);
  }

  private void configureTestBindings() {
    System.out.println("TEST BUTTON BINDINGS");

    Controlboard._xboxDrive.start().onTrue(_zeroGyroCommand);
    Controlboard._xboxDrive.back().onTrue(_zeroPoseCommand);

    Controlboard._xboxDrive.leftTrigger().toggleOnTrue(_visionGroundIntakeCommand);

    // /* -------- HOOK TESTING COMMANDS -------- */
    // Controlboard._xboxDrive.x().onTrue(_hookStowCommand);
    // Controlboard._xboxDrive.b().onTrue(_hookClimb);
    // Controlboard._xboxDrive.a().onTrue(_hookDeploy);
    // Controlboard._xboxDrive.y().onTrue(_climbRaiseToTrap);

    Controlboard._xboxDrive.x().onTrue(StaticScoreCommands.scoreSpeakerCommand(
        _drivetrain, _intake, _shooter, ShooterPivotPositions.AMP_FAR_SHOT, _leds
    ));
    Controlboard._xboxDrive.b().onTrue(StaticScoreCommands.scoreSpeakerCommand(
        _drivetrain, _intake, _shooter, ShooterPivotPositions.SOURCE_FAR_SHOT, _leds
    ));

    // Controlboard._xboxDrive.rightBumper().toggleOnTrue(_climberRaiseToCommand);

    // Controlboard.driverScoreAmpButton().toggleOnTrue(_scoreAmp2Command);
    // Controlboard._xboxDrive.rightTrigger().toggleOnTrue(_prepAmp);
    // Controlboard._xboxDrive.leftTrigger().toggleOnTrue(_ampScore);

    // Controlboard._xboxDrive.rightTrigger().toggleOnTrue(_prepToScore);
    // Controlboard._xboxDrive.leftTrigger().toggleOnTrue(_score);
    // Controlboard._xboxDrive.x().toggleOnTrue(_postScore);
    // Controlboard._xboxDrive.b().onTrue(_setPoseCommand);
    // Controlboard._xboxDrive.a().toggleOnTrue(_prepToIntake);
    // Controlboard._xboxDrive.y().toggleOnTrue(_intakeCommand);
    // Controlboard._xboxDrive.rightBumper().toggleOnTrue(_postIntake);
    // Controlboard._xboxDrive.leftTrigger().toggleOnTrue(_greifPass);

    // /* -------- INTAKE & BASIC TESTING SETUP -------- */
    // Controlboard.driverGroundIntakeButton().toggleOnTrue(_groundIntakeCommand);
    // Controlboard.driverScoreSpeakerSubwooferButton().toggleOnTrue(_scoreSubwooferCommand);

    // Controlboard._xboxDrive.rightBumper().onTrue(_sourceShooterIntake);
    Controlboard._xboxDrive.rightTrigger().onTrue(_sourceFeederIntake);

    // /* -------- INTAKE PIVOT BRINGUP -------- */
    // Controlboard._xboxDrive.x().onTrue(_intakeZero);
    // Controlboard._xboxDrive.y().onTrue(_intakeGround);
    // Controlboard._xboxDrive.a().onTrue(_intakeStow);
    // Controlboard._xboxDrive.b().onTrue(_intakeClear);

    // /* -------- SHOOTER PIVOT BRINGUP -------- */
    // Controlboard._xboxDrive.x().onTrue(_shooterAmp);
    // Controlboard._xboxDrive.y().onTrue(_shooterZero);
    // Controlboard._xboxDrive.a().onTrue(_shooterGround);
    // Controlboard._xboxDrive.b().onTrue(_shooterClear);
    // Controlboard._xboxDrive.rightBumper().onTrue(_shooterStow);

    // /* -------- SAFE PIVOT BRINGUP -------- */
    // Controlboard._xboxDrive.x().onTrue(_zone0);
    // Controlboard._xboxDrive.y().onTrue(_zone1);
    // Controlboard._xboxDrive.a().onTrue(_zero);
    // Controlboard._xboxDrive.b().onTrue(_shooterTest2);
    // Controlboard._xboxDrive.rightBumper().onTrue(_zone5);

    // Controlboard._xboxDrive.x().onTrue(_drive0.repeatedly());
    // Controlboard._xboxDrive.b().onTrue(_drive1.repeatedly());
    // Controlboard._xboxDrive.a().onTrue(_driveOpp1.repeatedly());

    // Controlboard._xboxDrive.rightTrigger().toggleOnTrue(_scoreSpeakerCommand);
    // Controlboard._xboxDrive.leftTrigger().toggleOnTrue(_ejectCommand); // no
    // vision ground
    // intake yet
    // Controlboard._xboxDrive.rightBumper().whileTrue(_scoreAmpCommand);
    // Controlboard._xboxDrive.leftBumper().toggleOnTrue(_groundIntakeCommand);

    // Controlboard._xboxDrive.a().onTrue(_scoreSpeakerCommand2);
    // Controlboard._xboxDrive.b().onTrue(_scoreSpeakerCommand3);
    // Controlboard._xboxDrive.y().toggleOnTrue(_scoreSpeakerCommand4);

    // Controlboard._xboxDrive.b().onTrue(_ejectCommand);

    // Controlboard._xboxDrive.y().onTrue(_climberExtendCommand);
    // Controlboard._xboxDrive.a().onTrue(_climberStowCommand);

    // Controlboard._xboxDrive.a().onTrue(_stowCommand);

    // Controlboard._xboxDrive.b().onTrue(_zero);
    // Controlboard._xboxDrive.b().onTrue(_scoreSubwooferCommand);

    // Controlboard._xboxDrive.b().onTrue(_climberRaiseToCommand);

    // Shooter pivot
    // Controlboard._xboxDrive.a().onTrue(_shooterAmp);
    // Controlboard._xboxDrive.b().onTrue(_shooterZero);

    // Controlboard._xboxDrive.rightTrigger().onTrue(_shooterRotateXCommand);

    // Controlboard._xboxDrive.x().onTrue(_feedUntilLimit);

    // Controlboard._xboxDrive.rightTrigger().onTrue(_eject);

    // Controlboard._xboxDrive.x().onTrue(_intakeGroundPivot); // intake to Ground
    // Controlboard._xboxDrive.y().onTrue(_shooterTest2); // intake to 30 deg
    // Controlboard._xboxDrive.b().onTrue(_shooterTest); // intake to Ground
    // Controlboard._xboxDrive.a().onTrue(_climberRaiseToCommand);
    // Controlboard._xboxDrive.b().onTrue(_climberRaiseToCommand2);
    // Controlboard._xboxDrive.a().whileTrue(_intakeRollerFF); // shoot
    // Controlboard._xboxDrive.a().whileTrue(_flywheelRollerFF); // shoot

    // Controlboard._xboxDrive.y().onTrue(_flywheelPowerCommand);

    // Controlboard._xboxDrive.b().onTrue(_flywheelRPSCommand);

    // // Leds
    // Controlboard._xboxDrive.a().onTrue(_ledCommandA);
    // Controlboard._xboxDrive.b().onTrue(_ledCommandB);
    // Controlboard._xboxDrive.x().onTrue(_ledCommandX);
    // Controlboard._xboxDrive.y().onTrue(_ledCommandY);

    // // Climber
    // Controlboard._xboxDrive.a().onTrue(_climberRaiseToCommand);
    // Controlboard._xboxDrive.b().onTrue(_climberRaiseToCommand2);
    // Controlboard._xboxDrive.x().onTrue(_climberRaiseToCommand3);
  }
}
