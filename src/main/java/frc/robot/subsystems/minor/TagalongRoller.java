package frc.robot.subsystems.minor;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.base.RollerRotateXCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotAltModes;
import frc.robot.parsers.RollerParser;
import frc.robot.parsers.json.utils.RollerConfJson;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.RollerAugment;
import frc.robot.utils.TagalongMinorSystemBase;
import frc.robot.utils.TagalongMinorSystemInterface;
import frc.robot.utils.TagalongTrapezoidProfile;

public class TagalongRoller
    extends TagalongMinorSystemBase implements TagalongMinorSystemInterface {
  protected final RollerParser _rollerParser;
  protected final RollerConfJson _rollerConf;
  public boolean _isRollerDisabled = false;

  /* -------- Hardware: motors and sensors -------- */
  protected TalonFX _rollerMotor;
  protected TalonFXConfiguration _rollerMotorConfig;
  protected Slot0Configs _rollerMotorSlot0;

  /* -------- Control: states and constants -------- */
  protected TagalongTrapezoidProfile.State _rollerCurState = new TagalongTrapezoidProfile.State();
  protected TagalongTrapezoidProfile.State _rollerGoalState = new TagalongTrapezoidProfile.State();
  protected boolean _followRollerProfile = false;
  private final double _rollerGearRatio;
  public final double _defaultRollerLowerToleranceRot;
  public final double _defaultRollerUpperToleranceRot;

  /* -------- Control: controllers and utilities -------- */
  protected SimpleMotorFeedforward _rollerFF;
  protected TagalongTrapezoidProfile _rollerProfile;
  protected PositionVoltage _requestedRollerPositionVoltage = new PositionVoltage(0.0);
  private VelocityVoltage _requestedRollerVelocityVoltage = new VelocityVoltage(0.0).withSlot(0);
  protected Timer _rollerTimer = new Timer();
  protected double _rollerGoalPositionRot;
  // protected double _rollerGoalVelocityRPS;

  /*-------Shuffleboard Entries------- */
  protected GenericSubscriber _rollerPFactorEntry, _rollerIFactorEntry, _rollerDFactorEntry;
  protected GenericSubscriber _rollerKSEntry, _rollerKVEntry, _rollerKAEntry;
  protected GenericPublisher _rollerCurrentPositionRotEntry, _rollerTargetPositionRotEntry;
  protected GenericPublisher _rollerCurrentVelocityRPSEntry, _pubRollerTargetVelocityRPSEntry;
  protected GenericPublisher _rollerTargetVelocityRPSEntry;

  public TagalongRoller(RollerParser parser) {
    super(parser);
    _rollerParser = parser;

    if (_configuredMinorSystemDisable) {
      _rollerGearRatio = 1.0;
      _rollerConf = null;
      _defaultRollerLowerToleranceRot = 0.0;
      _defaultRollerUpperToleranceRot = 0.0;
      return;
    }

    _rollerConf = parser.rollerConf;
    _rollerProfile = new TagalongTrapezoidProfile(
        _rollerConf.trapezoidalLimits.getLimitsRot(),
        new TagalongTrapezoidProfile.State(0.0, 0.0),
        new TagalongTrapezoidProfile.State(0.0, 0.0)
    );

    _rollerGearRatio = _rollerConf.getGearRatio();
    _rollerMotor = _rollerConf.rollerMotor.getTalonFX();
    _rollerMotorConfig = _rollerConf.rollerMotorControl.getFullMotorConfiguration();
    _rollerMotorSlot0 = _rollerConf.rollerMotorControl.getSlot0();

    _rollerFF = _rollerConf.feedforward.getSimpleMotorFeedforward();
    _defaultRollerLowerToleranceRot = _rollerConf.defaultTolerances.getLowerToleranceRot();
    _defaultRollerUpperToleranceRot = _rollerConf.defaultTolerances.getUpperToleranceRot();

    configRollerMotor();
  }

  @Override
  public void onEnable() {
    if (_isMinorSystemDisabled) {
      return;
    }

    if (RobotAltModes.isPIDTuningMode && RobotAltModes.isRollerTuningMode) {
      _rollerMotorSlot0.kP = _rollerPFactorEntry.getDouble(_rollerMotorSlot0.kP);
      _rollerMotorSlot0.kI = _rollerIFactorEntry.getDouble(_rollerMotorSlot0.kI);
      _rollerMotorSlot0.kD = _rollerDFactorEntry.getDouble(_rollerMotorSlot0.kD);
      _rollerMotor.getConfigurator().apply(_rollerMotorSlot0);
    }

    if (RobotAltModes.isRollerTuningMode) {
      _rollerFF = new SimpleMotorFeedforward(
          _rollerKSEntry.getDouble(_rollerFF.ks),
          _rollerKVEntry.getDouble(_rollerFF.kv),
          _rollerKAEntry.getDouble(_rollerFF.ka)
      );
    }
  }

  @Override
  public void onDisable() {
    if (_isMinorSystemDisabled) {
      return;
    }
    setRollerFollowProfile(false); // TODO move to telopDisable function
  }

  public void periodic() {
    if (_isMinorSystemDisabled) {
      return;
    }

    // if (_rollerConf.name.equalsIgnoreCase("Intake RP")) {
    //   // System.out.println(_rollerKSEntry.getDouble(_rollerFF.ks));
    //   System.out.println(
    //       "rollerff " + _rollerConf.name
    //       + _rollerFF.calculate(
    //           0.001,
    //           Units.rotationsToRadians(
    //               ((_rollerTargetVelocityRPSEntry.getDouble(0.0)
    //                 - _rollerMotor.getVelocity().getValueAsDouble())
    //                / Constants.LOOP_PERIOD_S)
    //           )
    //       )
    //   );
    //   // // TODO: Remove -- only for testing remove after tuning kS
    // _rollerMotor.getMotorVoltage();
    if (_followRollerProfile) {
      followLastRollerProfile();
    }
  }

  @Override
  public void disabledPeriodic() {
    if (_isMinorSystemDisabled) {
      return;
    }
  }

  @Override
  public void simulationInit() {
    if (_isMinorSystemDisabled) {
      return;
    }
  }

  @Override
  public void simulationPeriodic() {
    if (_isMinorSystemDisabled) {
      return;
    }
  }

  // TODO IMPLEMENT
  @Override
  public boolean motorResetConfig() {
    if (_isMinorSystemDisabled) {
      return false;
    }
    return super.motorResetConfig();
  }

  // TODO IMPLEMENT
  @Override
  public boolean checkInitStatus() {
    return super.checkInitStatus();
  }

  @Override
  public void updateShuffleboard() {
    if (_isMinorSystemDisabled) {
      return;
    }

    if (RobotAltModes.isRollerTuningMode) {
      _rollerCurrentPositionRotEntry.setDouble(getRollerMotorPosition());
      _rollerTargetPositionRotEntry.setDouble(_rollerGoalPositionRot);
      _rollerCurrentVelocityRPSEntry.setDouble(_rollerMotor.getVelocity().getValueAsDouble());
    }
  }

  @Override
  public void configShuffleboard() {
    if (_isMinorSystemDisabled) {
      return;
    }

    ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning tab");
    ShuffleboardLayout rollerLayout =
        tuningTab.getLayout(_rollerConf.name + "Rollers", BuiltInLayouts.kGrid)
            .withSize(3, 4)
            .withPosition(2 * Controlboard.get()._tuningTabCounter++, 0);
    if (RobotAltModes.isRollerTuningMode) {
      _rollerCurrentPositionRotEntry = rollerLayout.add(_rollerConf.name + "Current Position", 0.0)
                                           .withPosition(2, 0)
                                           .getEntry();
      _rollerTargetPositionRotEntry =
          rollerLayout.add(_rollerConf.name + "Target Position", 0.0).withPosition(2, 1).getEntry();
      _rollerCurrentVelocityRPSEntry = rollerLayout.add(_rollerConf.name + "Current Velocity", 0.0)
                                           .withPosition(2, 1)
                                           .getEntry();
      _rollerTargetVelocityRPSEntry =
          rollerLayout.add(_rollerConf.name + "Target Velocity", 0.0).withPosition(2, 3).getEntry();
      _rollerKSEntry = rollerLayout.add(_rollerConf.name + " kS", _rollerConf.feedforward.s)
                           .withPosition(1, 1)
                           .getEntry();
      _rollerKVEntry = rollerLayout.add(_rollerConf.name + " kV", _rollerConf.feedforward.v)
                           .withPosition(1, 2)
                           .getEntry();
      _rollerKAEntry = rollerLayout.add(_rollerConf.name + " kA", _rollerConf.feedforward.a)
                           .withPosition(1, 3)
                           .getEntry();
    }

    if (RobotAltModes.isPIDTuningMode && RobotAltModes.isRollerTuningMode) {
      _rollerPFactorEntry =
          rollerLayout.add(_rollerConf.name + "P Fac", 0.0).withPosition(0, 0).getEntry();
      _rollerIFactorEntry =
          rollerLayout.add(_rollerConf.name + "I Fac", 0.0).withPosition(0, 1).getEntry();
      _rollerDFactorEntry =
          rollerLayout.add(_rollerConf.name + "D Fac", 0.0).withPosition(0, 2).getEntry();
    }
  }

  public double motorToRollerRot(double motorRot) {
    if (_isMinorSystemDisabled) {
      return 0.0;
    }

    return motorRot / _rollerGearRatio;
  }

  public double rollerRotToMotor(double rollerRot) {
    if (_isMinorSystemDisabled) {
      return 0.0;
    }

    return rollerRot * _rollerGearRatio;
  }

  public void followLastRollerProfile() {
    if (_isMinorSystemDisabled) {
      return;
    }

    TagalongTrapezoidProfile.State nextState = _rollerProfile.calculate(_rollerTimer.get());

    _rollerMotor.setControl(
        _requestedRollerPositionVoltage.withPosition(rollerRotToMotor(nextState.position))
            .withFeedForward(_rollerFF.calculate(
                nextState.velocity,
                (nextState.velocity - _rollerCurState.velocity) / Constants.LOOP_PERIOD_S
            ))
    );

    if (RobotAltModes.isRollerTuningMode) {
      _rollerTargetVelocityRPSEntry.setDouble(nextState.velocity);
    }

    _rollerCurState = nextState;
  }

  public void setRollerProfile(double goalPositionRot, double goalVelocityRPS) {
    if (_isMinorSystemDisabled) {
      return;
    }
    _rollerGoalState.position = goalPositionRot;
    _rollerGoalState.velocity = goalVelocityRPS;
    _rollerCurState = _rollerProfile.calculate(_rollerTimer.get());
    _rollerCurState.position = getRollerPosition();

    _rollerProfile = new TagalongTrapezoidProfile(
        _rollerConf.trapezoidalLimits.getLimitsRot(), _rollerGoalState, _rollerCurState
    );
    _rollerTimer.restart();
  }

  public boolean isRollerProfileFinished() {
    return _isMinorSystemDisabled
        || (_rollerCurState.position == _rollerGoalState.position
            && _rollerCurState.velocity == _rollerGoalState.velocity);
  }

  public void configRollerMotor() {
    if (_isMinorSystemDisabled) {
      return;
    }

    _rollerMotor.getConfigurator().apply(_rollerMotorConfig);
  }

  public void setRollerPower(double power) {
    if (!_isMinorSystemDisabled)
      _rollerMotor.set(power);
  }

  public double getRollerPower() {
    return (_isMinorSystemDisabled) ? 0.0 : _rollerMotor.get();
  }

  public TalonFX getRollerMotor() {
    if (_isMinorSystemDisabled) {
      return new TalonFX(0);
    }
    return _rollerMotor;
  }

  public double getRollerMotorPosition() {
    if (_isMinorSystemDisabled) {
      return 0.0;
    }
    return _rollerMotor.getPosition().getValueAsDouble();
  }

  public double getRollerPosition() {
    if (_isMinorSystemDisabled) {
      return 0.0;
    }
    return motorToRollerRot(getRollerMotorPosition());
  }

  public void setRollerFollowProfile(boolean followProfile) {
    if (!(_isMinorSystemDisabled)) {
      _followRollerProfile = followProfile;
    }
  }

  public void setRollerVelocityControl(double rps, boolean withFF) {
    if (_isMinorSystemDisabled) {
      return;
    }

    _followRollerProfile = false;
    _rollerMotor.setControl(_requestedRollerVelocityVoltage.withVelocity(rps).withFeedForward(
        withFF ? calculateFeedforward(rps) : 0.0
    ));
  }

  public double calculateFeedforward(double rps) {
    return calculateFeedforward(rps, 0.0);
  }

  public double calculateFeedforward(double rps, double acceleration) {
    if (_isMinorSystemDisabled) {
      return 0.0;
    }

    return _rollerFF.calculate(rps, acceleration);
  }

  public void disableRoller(boolean disable) {
    _isRollerDisabled = _isMinorSystemDisabled || disable;
  }

  public Command setRollerPowerCommand(double power) {
    return new InstantCommand(() -> setRollerPower(power));
  }

  public Command setRollerRPSCommand(double rps) {
    return new InstantCommand(() -> setRollerVelocityControl(rps, false));
  }

  public Command setRollerRPSWithFFCommand(double rps) {
    return new InstantCommand(() -> setRollerVelocityControl(rps, true));
  }

  public Command startEndRollerPowerCommand(double power) {
    return (Commands.startEnd(() -> setRollerPower(power), () -> setRollerPower(0.0)));
  }

  public Command startEndRollerRPSCommand(double rps) {
    return Commands.startEnd(() -> setRollerVelocityControl(rps, false), () -> setRollerPower(0.0));
  }

  public Command startEndRollerRPSWithFFCommand(double rps) {
    return Commands.startEnd(() -> setRollerVelocityControl(rps, true), () -> setRollerPower(0.0));
  }
}
