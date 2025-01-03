package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Robot;
import frc.robot.constants.confs.BaseIntakeConf;
import frc.robot.utils.IntakeIOInputsAutoLogged;
import frc.robot.utils.IntakeIOTalonFX;
import org.littletonrobotics.junction.Logger;
import tagalong.subsystems.TagalongSubsystemBase;
import tagalong.subsystems.micro.Pivot;
import tagalong.subsystems.micro.Roller;
import tagalong.subsystems.micro.augments.PivotAugment;
import tagalong.subsystems.micro.augments.RollerAugment;

public class Intake extends TagalongSubsystemBase implements RollerAugment, PivotAugment {
  public static class IntakeConstants {
    public static final double ROLLER_IN_RPS = 75.0;
    public static final double ROLLER_OUT_RPS = -30.0;
    public static final double ROLLER_EJECT_RPS = -40.0;
  }

  public final BaseIntakeConf _intakeConf;

  private final Pivot _pivot;
  private final Roller _roller;

  protected GenericPublisher _inIntakeEntry;

  /* -------- Hardware: motors and sensors -------- */
  // private DigitalInput _intakeBreakBeam;

  /* -------- Logging: utilities and configs -------- */
  private IntakeIOTalonFX _io;
  private final IntakeIOInputsAutoLogged _inputs = new IntakeIOInputsAutoLogged();

  /* -------- Control: states and constants -------- */
  public final double _pivotUnsafeMinRot;
  public final double _pivotUnsafeMaxRot;

  public Intake(BaseIntakeConf intakeConf) {
    super(intakeConf);
    _pivot = new Pivot(intakeConf.pivotConf);
    _roller = new Roller(intakeConf.rollerConf);
    // _intakeBreakBeam = new DigitalInput(_intakeConf.breakBeamChannel);

    if (_pivot._configuredMicrosystemDisable || _roller._configuredMicrosystemDisable) {
      _pivotUnsafeMinRot = 0.0;
      _pivotUnsafeMaxRot = 0.0;
      _intakeConf = null;
      return;
    }
    _intakeConf = intakeConf;
    _pivotUnsafeMinRot = intakeConf.pivotUnsafePositionalMin;
    _pivotUnsafeMaxRot = intakeConf.pivotUnsafePositionalMax;

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

    _pivot.setPivotPower(0.0);
    _pivot.setFollowProfile(true);
    _roller.setRollerPower(0.0);
    _pivot.getPrimaryMotor().setNeutralMode(NeutralModeValue.Brake);
    _pivot.setPivotProfile(_pivot.getPivotPosition(), 0.0);
    _pivot.getPrimaryMotor().setControl(new PositionVoltage(_pivot.getPivotPosition()));
  }

  @Override
  public void onDisable() {
    if (_isSubsystemDisabled) {
      return;
    }
    _pivot.onDisable();
    _roller.onDisable();

    _pivot.setPivotPower(0.0);
    _pivot.setFollowProfile(true);
    _roller.setRollerPower(0.0);
    _pivot.getPrimaryMotor().setNeutralMode(NeutralModeValue.Brake);
    _pivot.setPivotProfile(_pivot.getPivotPosition(), 0.0);
    _pivot.getPrimaryMotor().setControl(new PositionVoltage(_pivot.getPivotPosition()));
  }

  @Override
  public void periodic() {
    if (_isSubsystemDisabled) {
      return;
    }
    // System.out.println("intake pivot position" +
    // getPivot()._pivotCancoder.getPosition());
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
    if (Robot.isReal()) {
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
  public Pivot getPivot() {
    return _pivot;
  }

  @Override
  public Pivot getPivot(int i) {
    return _pivot;
  }

  @Override
  public Roller getRoller() {
    return _roller;
  }

  @Override
  public Roller getRoller(int i) {
    return _roller;
  }

  public boolean isPieceInIntake() {
    if (_isSubsystemDisabled) {
      return false;
    }
    // System.out.println(
    // "supply current" +
    // getRoller().getPrimaryMotor().getSupplyCurrent().getValueAsDouble()
    // + "stator current" +
    // getRoller().getPrimaryMotor().getStatorCurrent().getValueAsDouble()
    // );
    return getRoller().getPrimaryMotor().getSupplyCurrent().getValueAsDouble() > 20.0
        || getRoller().getPrimaryMotor().getStatorCurrent().getValueAsDouble() > 20.0;
  }
}
