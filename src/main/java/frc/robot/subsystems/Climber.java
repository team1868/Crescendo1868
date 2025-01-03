package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.confs.BaseClimberConf;
import frc.robot.utils.ClimberIOInputsAutoLogged;
import frc.robot.utils.ClimberIOTalonFX;
import org.littletonrobotics.junction.Logger;
import tagalong.subsystems.TagalongSubsystemBase;
import tagalong.subsystems.micro.Elevator;
import tagalong.subsystems.micro.Roller;
import tagalong.subsystems.micro.augments.ElevatorAugment;
import tagalong.subsystems.micro.augments.RollerAugment;

public class Climber extends TagalongSubsystemBase implements ElevatorAugment, RollerAugment {
  public static final class ElevatorConstants {
    public static final int ELEVATOR_ID = 0;
    public static final int HOOKS_ID = 1;

    public static final double ELEVATOR_ZEROING_SPEED_MPS = -0.06;
    public static final double ELEVATOR_PREP_SPEED_MPS = -0.01;
    public static final double ELEVATOR_ZEROING_STALL_TOLERANCE = 50;
    public static final int ELEVATOR_ZEROING_STALL_LOOPS = 6;

    public static final double ELEVATOR_AMP_MPS = 1.0;
    public static final double ELEVATOR_TRAP_MPS = 1.0;
  }

  public static final class RollerConstants { public static final double ROLLER_AMP_SHOT = 50.0; }

  public final BaseClimberConf _climberConf;

  private final Elevator _elevator;
  private final Elevator _hooks;
  private final Roller _roller;

  /* -------- Logging: utilities and configs -------- */
  private final ClimberIOTalonFX _io;
  private final ClimberIOInputsAutoLogged _inputs = new ClimberIOInputsAutoLogged();

  @Override
  public Elevator getElevator() {
    return _elevator;
  }

  @Override
  public Elevator getElevator(int i) {
    switch (i) {
      case 1:
        return _hooks;
      case 0:
      default:
        return _elevator;
    }
  }

  @Override
  public Roller getRoller() {
    return _roller;
  }

  @Override
  public Roller getRoller(int i) {
    return _roller;
  }

  public Climber(BaseClimberConf climberConf) {
    super(climberConf);
    _elevator = new Elevator(climberConf.elevatorConf);
    _hooks = new Elevator(climberConf.hooksConf);
    _roller = new Roller(climberConf.rollerConf);

    if (_elevator._configuredMicrosystemDisable || _hooks._configuredMicrosystemDisable
        || _roller._configuredMicrosystemDisable) {
      _io = null;
      _climberConf = null;
      setDisabled(false);
      return;
    }

    _climberConf = climberConf;

    int counter = 0;
    while (!checkInitStatus() && counter < 100) {
      System.out.println("Waiting for climber");
    }
    if (counter >= 100) {
      System.out.println("failed to init");
    }

    // THIS HAS TO GO LAST
    _io = new ClimberIOTalonFX(this);
    configShuffleboard();
  }

  public boolean isReadyToClimb() {
    return _isSubsystemDisabled ? true : false;
  }

  public boolean isClimbed() {
    return _isSubsystemDisabled ? true : false;
  }

  @Override
  public void onEnable() {
    if (_isSubsystemDisabled) {
      return;
    }
    _elevator.onEnable();
    _roller.onEnable();
    _hooks.onEnable();
    // for testing
    // _elevator.getPrimaryMotor().setControl(
    // new VelocityVoltage(0.000001).withFeedForward(_elevator._elevatorFF.ks)
    // );
    // System.out.println("ks " + _elevator._elevatorFF.ks);
    _elevator.setPrimaryPower(0.0);
    _elevator.setFollowProfile(true);
    _elevator.getPrimaryMotor().setNeutralMode(NeutralModeValue.Brake);

    _hooks.setPrimaryPower(0.0);
    _hooks.setFollowProfile(false);
    _hooks.getPrimaryMotor().setNeutralMode(NeutralModeValue.Brake);

    _elevator.setElevatorProfile(_elevator.getElevatorHeightM(), 0.0);
    _elevator.getPrimaryMotor().setControl(new PositionVoltage(_elevator.getPrimaryMotorPosition())
    );

    _hooks.setElevatorProfile(_hooks.getElevatorHeightM(), 0.0);
    _hooks.getPrimaryMotor().setControl(new PositionVoltage(_hooks.getPrimaryMotorPosition()));
  }

  public void onDisable() {
    if (_isSubsystemDisabled) {
      return;
    }
    _elevator.onDisable();
    _roller.onDisable();
    _hooks.onDisable();

    _elevator.setPrimaryPower(0.0);
    _elevator.setFollowProfile(true);
    _elevator.getPrimaryMotor().setNeutralMode(NeutralModeValue.Brake);

    _hooks.setPrimaryPower(0.0);
    _hooks.setFollowProfile(false);
    _hooks.getPrimaryMotor().setNeutralMode(NeutralModeValue.Brake);

    _elevator.setElevatorProfile(_elevator.getElevatorHeightM(), 0.0);
    _elevator.getPrimaryMotor().setControl(new PositionVoltage(_elevator.getPrimaryMotorPosition())
    );

    _hooks.setElevatorProfile(_hooks.getElevatorHeightM(), 0.0);
    _hooks.getPrimaryMotor().setControl(new PositionVoltage(_hooks.getPrimaryMotorPosition()));
  }

  @Override
  public void periodic() {
    if (_isSubsystemDisabled) {
      return;
    }
    _elevator.periodic();
    _roller.periodic();
    _hooks.periodic();

    // Logging
    _io.updateInputs(_inputs);
    Logger.processInputs("Climber", _inputs);
    updateShuffleboard();
  }

  @Override
  public void simulationInit() {
    _elevator.simulationInit();
    _roller.simulationInit();
    _hooks.simulationInit();
  }

  @Override
  public void simulationPeriodic() {
    _elevator.simulationPeriodic();
    _roller.simulationPeriodic();
    _hooks.simulationPeriodic();
  }

  @Override
  public void updateShuffleboard() {
    _elevator.updateShuffleboard();
    _roller.updateShuffleboard();
    _hooks.updateShuffleboard();
  }

  @Override
  public void configShuffleboard() {
    _elevator.configShuffleboard();
    _roller.configShuffleboard();
    _hooks.configShuffleboard();
  }

  // TODO IMPLEMENT
  @Override
  public boolean checkInitStatus() {
    return super.checkInitStatus() && _elevator.checkInitStatus() && _roller.checkInitStatus()
        && _hooks.checkInitStatus();
  }
}
