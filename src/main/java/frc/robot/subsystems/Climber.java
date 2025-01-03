package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.parsers.ClimberParser;
import frc.robot.parsers.json.ClimberConfJson;
import frc.robot.subsystems.minor.TagalongElevator;
import frc.robot.subsystems.minor.TagalongRoller;
import frc.robot.utils.ClimberIOInputsAutoLogged;
import frc.robot.utils.ClimberIOTalonFX;
import frc.robot.utils.ElevatorAugment;
import frc.robot.utils.RollerAugment;
import frc.robot.utils.TagalongSubsystemBase;
import org.littletonrobotics.junction.Logger;

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

  private final TagalongElevator _elevator;
  private final TagalongElevator _hooks;
  private final TagalongRoller _roller;
  public final ClimberParser _climberParser;
  public final ClimberConfJson _climberConf;

  /* -------- Logging: utilities and configs -------- */
  private final ClimberIOTalonFX _io;
  private final ClimberIOInputsAutoLogged _inputs = new ClimberIOInputsAutoLogged();

  @Override
  public TagalongElevator getElevator() {
    return _elevator;
  }

  @Override
  public TagalongElevator getElevator(int i) {
    switch (i) {
      case 1:
        return _hooks;
      case 0:
      default:
        return _elevator;
    }
  }

  @Override
  public TagalongRoller getRoller() {
    return _roller;
  }

  @Override
  public TagalongRoller getRoller(int i) {
    return _roller;
  }

  public Climber(String filePath) {
    this(filePath == null ? null : new ClimberParser(Filesystem.getDeployDirectory(), filePath));
  }

  public Climber(ClimberParser parser) {
    super(parser);
    _climberParser = parser;

    if (_configuredDisable) {
      _io = null;
      _climberConf = null;
      _elevator = new TagalongElevator(null);
      _hooks = new TagalongElevator(null);
      _roller = new TagalongRoller(null);
      return;
    }

    _climberConf = _climberParser.climberConf;
    _elevator = new TagalongElevator(parser.elevatorParser);
    _hooks = new TagalongElevator(parser.hookParser);
    _roller = new TagalongRoller(parser.rollerParser);

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
    // _elevator.getElevatorMotor().setControl(
    //     new VelocityVoltage(0.000001).withFeedForward(_elevator._elevatorFF.ks)
    // );
    // System.out.println("ks " + _elevator._elevatorFF.ks);
  }

  public void onDisable() {
    if (_isSubsystemDisabled) {
      return;
    }
    _elevator.onDisable();
    _roller.onDisable();
    _hooks.onDisable();
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
  public void disabledPeriodic() {
    _elevator.disabledPeriodic();
    _roller.disabledPeriodic();
    _hooks.disabledPeriodic();
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
