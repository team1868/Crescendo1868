package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.RobotAltModes;
import frc.robot.parsers.IntakeParser;
import frc.robot.parsers.json.IntakeConfJson;
import frc.robot.subsystems.minor.TagalongPivot;
import frc.robot.subsystems.minor.TagalongRoller;
import frc.robot.utils.IntakeIOInputsAutoLogged;
import frc.robot.utils.IntakeIOTalonFX;
import frc.robot.utils.PivotAugment;
import frc.robot.utils.RollerAugment;
import frc.robot.utils.TagalongSubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends TagalongSubsystemBase implements RollerAugment, PivotAugment {
  public static class IntakeConstants {
    public static final double ROLLER_IN_RPS = 75.0;
    public static final double ROLLER_OUT_RPS = -30.0;
    public static final double ROLLER_EJECT_RPS = -40.0;
  }

  public final IntakeParser _intakeParser;
  public final IntakeConfJson _intakeConf;

  private final TagalongPivot _pivot;
  private final TagalongRoller _roller;

  protected GenericPublisher _inIntakeEntry;

  /* -------- Hardware: motors and sensors -------- */
  // private DigitalInput _intakeBreakBeam;

  /* -------- Logging: utilities and configs -------- */
  private IntakeIOTalonFX _io;
  private final IntakeIOInputsAutoLogged _inputs = new IntakeIOInputsAutoLogged();

  /* -------- Control: states and constants -------- */
  public final double _pivotUnsafeMinRot;
  public final double _pivotUnsafeMaxRot;

  public Intake(String filePath) {
    this(filePath == null ? null : new IntakeParser(Filesystem.getDeployDirectory(), filePath));
  }

  public Intake(IntakeParser conf) {
    super(conf);
    _intakeParser = conf;

    if (_configuredDisable) {
      _intakeConf = null;
      _pivotUnsafeMinRot = 0.0;
      _pivotUnsafeMaxRot = 0.0;
      _pivot = new TagalongPivot(null);
      _roller = new TagalongRoller(null);
      return;
    }

    _intakeConf = _intakeParser.intakeConf;
    _pivot = new TagalongPivot(_intakeParser.pivotParser);
    _roller = new TagalongRoller(_intakeParser.rollerParser);
    // _intakeBreakBeam = new DigitalInput(_intakeConf.breakBeamChannel);

    _pivotUnsafeMinRot = _intakeConf.pivotUnsafePositionalLimits.getMinRot();
    _pivotUnsafeMaxRot = _intakeConf.pivotUnsafePositionalLimits.getMaxRot();

    int counter = 0;
    while (!checkInitStatus() && counter < 100) {
      System.out.println("Waiting for Intake");
    }

    if (counter >= 100) {
      System.out.println("failed to init Intake");
    }

    // THIS HAS TO GO LAST
    _io = new IntakeIOTalonFX(this);
    configShuffleboard();
  }

  @Override
  public void onEnable() {
    if (_isSubsystemDisabled) {
      return;
    }
    _pivot.onEnable();
    _roller.onEnable();
  }

  @Override
  public void onDisable() {
    if (_isSubsystemDisabled) {
      return;
    }
    _pivot.onDisable();
    _roller.onDisable();
  }

  @Override
  public void periodic() {
    if (_isSubsystemDisabled) {
      return;
    }
    // System.out.println("intake pivot position" + getPivot()._pivotCancoder.getPosition());
    updateShuffleboard();
    _pivot.periodic();
    _roller.periodic();
    // Logging
    _io.updateInputs(_inputs);
    Logger.processInputs("Intake", _inputs);
  }

  // TODO IMPLEMENT
  @Override
  public boolean checkInitStatus() {
    return super.checkInitStatus() && _roller.checkInitStatus() && _pivot.checkInitStatus();
  }

  // TODO IMPLEMENT
  @Override
  public void simulationInit() {
    if (_isSubsystemDisabled) {
      return;
    }
    _pivot.simulationInit();
    _roller.simulationInit();
  }

  @Override
  public void simulationPeriodic() {
    if (_isSubsystemDisabled) {
      return;
    }
    if (!RobotAltModes.isSim) {
      return;
    }
    _pivot.simulationPeriodic();
    _roller.simulationPeriodic();
  }

  // TODO IMPLEMENT
  @Override
  protected void updateShuffleboard() {
    if (_isSubsystemDisabled) {
      return;
    }
    _pivot.updateShuffleboard();
    _roller.updateShuffleboard();

    _inIntakeEntry.setBoolean(isPieceInIntake());
  }

  // TODO IMPLEMENT
  @Override
  protected void configShuffleboard() {
    if (_isSubsystemDisabled) {
      return;
    }
    _pivot.configShuffleboard();
    _roller.configShuffleboard();

    ShuffleboardTab feedbackTab = Shuffleboard.getTab("Feedback tab");
    _inIntakeEntry = feedbackTab.add("In Intake", isPieceInIntake())
                         .withPosition(0, 0)
                         .withSize(4, 4)
                         .getEntry();
    ;
  }

  @Override
  public void disabledPeriodic() {
    if (_isSubsystemDisabled) {
      return;
    }
    _pivot.disabledPeriodic();
    _roller.disabledPeriodic();
  }

  @Override
  public TagalongPivot getPivot() {
    return _pivot;
  }

  @Override
  public TagalongPivot getPivot(int i) {
    return _pivot;
  }

  @Override
  public TagalongRoller getRoller() {
    return _roller;
  }

  @Override
  public TagalongRoller getRoller(int i) {
    return _roller;
  }

  public boolean isPieceInIntake() {
    if (_isSubsystemDisabled) {
      return false;
    }
    // System.out.println(
    //     "supply current" + getRoller().getRollerMotor().getSupplyCurrent().getValueAsDouble()
    //     + "stator current" + getRoller().getRollerMotor().getStatorCurrent().getValueAsDouble()
    // );
    return getRoller().getRollerMotor().getSupplyCurrent().getValueAsDouble() > 20.0
        || getRoller().getRollerMotor().getStatorCurrent().getValueAsDouble() > 20.0;
  }
}
