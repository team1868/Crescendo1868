package frc.robot.subsystems.minor;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.RobotAltModes;
import frc.robot.constants.enums.IntakePivotPositions;
import frc.robot.constants.enums.ShooterPivotPositions;
import frc.robot.parsers.FlywheelParser;
import frc.robot.parsers.json.utils.FlywheelConfJson;
import frc.robot.subsystems.Controlboard;
import frc.robot.utils.TagalongMinorSystemBase;
import frc.robot.utils.TagalongMinorSystemInterface;

public class TagalongDualMotorFlywheel
    extends TagalongMinorSystemBase implements TagalongMinorSystemInterface {
  public final FlywheelParser _flywheelParser;
  public final FlywheelConfJson _flywheelConf;

  /* -------- Hardware: motors and sensors -------- */
  private TalonFX _flywheelMotor;
  private TalonFX _flywheelFollowerMotor;
  private TalonFXConfiguration _flywheelMotorConfig, _flywheelFollowerMotorConfig;
  protected Slot0Configs _flywheelMotorSlot0 = new Slot0Configs();

  /* -------- Control: states and constants -------- */
  private final double _flywheelGearRatio;
  private final VelocityVoltage _requestedFlywheelVelocityVoltage = new VelocityVoltage(0.0);

  /* --- Shuffleboard Entries --- */
  protected GenericSubscriber _flywheelPFactorEntry, _flywheelIFactorEntry, _flywheelDFactorEntry;
  protected GenericSubscriber _flywheelKSEntry, _flywheelKVEntry, _flywheelKAEntry;
  protected GenericSubscriber _targetVelocity;
  protected GenericPublisher _flywheelVelocity, _publishedTargetVelocity;

  protected SimpleMotorFeedforward _flywheelFF;

  public TagalongDualMotorFlywheel(FlywheelParser parser) {
    super(parser);
    _flywheelParser = parser;

    if (_configuredMinorSystemDisable) {
      _flywheelGearRatio = 1.0;
      _flywheelConf = null;
      return;
    }

    _flywheelConf = parser.flywheelConf;
    _flywheelMotor = _flywheelConf.flywheelMotor.getTalonFX();
    _flywheelFollowerMotor = _flywheelConf.flywheelFollowerMotor.getTalonFX();
    _flywheelGearRatio = _flywheelConf.getGearRatio();
    _flywheelFF = _flywheelConf.feedforward.getSimpleMotorFeedforward();

    _flywheelMotorConfig = _flywheelConf.flywheelMotorControl.getFullMotorConfiguration();
    // _flywheelMotorConfig.Voltage.PeakReverseVoltage = 0.0;
    _flywheelFollowerMotorConfig =
        _flywheelConf.flywheelFollowerControl.getFullMotorConfiguration();
    // _flywheelFollowerMotorConfig.Voltage.PeakReverseVoltage = 0.0;
    _flywheelMotorSlot0 = _flywheelConf.flywheelMotorControl.getSlot0();

    configFlywheelMotor();
  }

  @Override
  public void periodic() {
    if (_isMinorSystemDisabled) {
      return;
    }
  }

  @Override
  public void onEnable() {
    if (_isMinorSystemDisabled) {
      return;
    }
    if (RobotAltModes.isFlywheelTuningMode) {
      _flywheelFF = new SimpleMotorFeedforward(
          _flywheelKSEntry.getDouble(_flywheelFF.ks),
          _flywheelKVEntry.getDouble(_flywheelFF.kv),
          _flywheelKAEntry.getDouble(_flywheelFF.ka)
      );
      // For testing only
      // _flywheelMotor.setControl(
      //     _requestedFlywheelVelocityVoltage.withVelocity(1.0).withFeedForward(0)
      // );
    }

    if (RobotAltModes.isFlywheelTuningMode && RobotAltModes.isPIDTuningMode) {
      _flywheelMotorSlot0.kP = _flywheelPFactorEntry.getDouble(_flywheelMotorSlot0.kP);
      _flywheelMotorSlot0.kI = _flywheelIFactorEntry.getDouble(_flywheelMotorSlot0.kI);
      _flywheelMotorSlot0.kD = _flywheelDFactorEntry.getDouble(_flywheelMotorSlot0.kD);
      _flywheelMotor.getConfigurator().apply(_flywheelMotorSlot0);
      _flywheelFollowerMotor.getConfigurator().apply(_flywheelMotorSlot0);
    }
  }

  @Override
  public void onDisable() {
    if (_isMinorSystemDisabled)
      return;
  }

  private void configFlywheelMotor() {
    _flywheelMotor.getConfigurator().apply(_flywheelMotorConfig);
    _flywheelFollowerMotor.getConfigurator().apply(_flywheelFollowerMotorConfig);
    _flywheelFollowerMotor.setControl(new StrictFollower(_flywheelMotor.getDeviceID()));
  }

  public void setFlywheelPower(double power) {
    if (_isMinorSystemDisabled) {
      return;
    }
    _flywheelMotor.set(power);
  }

  public double getFlywheelPower() {
    return _isMinorSystemDisabled ? 0.0 : _flywheelMotor.get();
  }

  public double getFlywheelVelocity() {
    return _isMinorSystemDisabled ? 0.0 : _flywheelMotor.getVelocity().getValueAsDouble();
  }

  public boolean isFlywheelAtTargetSpeed(double targetSpeed) {
    if (_isMinorSystemDisabled) {
      return true;
    }
    // System.out.println(_flywheelMotor.getVelocity().getValueAsDouble());
    return inTolerance(
        _flywheelMotor.getVelocity().getValueAsDouble() - targetSpeed,
        -_flywheelConf.defaultTolerances.lowerTolerance,
        _flywheelConf.defaultTolerances.upperTolerance
    );
  }

  public void setFlywheelControl(double rps, boolean withFF) {
    if (_isMinorSystemDisabled) {
      return;
    }
    _flywheelMotor.setControl(_requestedFlywheelVelocityVoltage.withOverrideBrakeDurNeutral(withFF)
                                  .withVelocity(rps)
                                  .withFeedForward(withFF ? calculateFF(rps) : 0.0));
  }

  // TODO fix?
  public double calculateFF(double velocity) {
    if (_isMinorSystemDisabled) {
      return -1.0;
    }
    return _flywheelFF.calculate(velocity);
  }

  @Override
  public void disabledPeriodic() {
    if (_isMinorSystemDisabled) {
      return;
    }
  }

  // TODO IMPLEMENT
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

  @Override
  public void updateShuffleboard() {
    if (RobotAltModes.isFlywheelTuningMode) {
      _flywheelVelocity.setDouble(_flywheelMotor.getVelocity().getValueAsDouble());
      _publishedTargetVelocity.setDouble(_targetVelocity.getDouble(0));
    }
  }

  @Override
  public void configShuffleboard() {
    ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning tab");
    ShuffleboardLayout flywheelLayout =
        tuningTab.getLayout(_flywheelConf.name, BuiltInLayouts.kGrid)
            .withSize(3, 4)
            .withPosition(2 * Controlboard.get()._tuningTabCounter++, 0);
    if (RobotAltModes.isPIDTuningMode && RobotAltModes.isFlywheelTuningMode) {
      _flywheelPFactorEntry =
          flywheelLayout.add(_flywheelConf.name + " P Fac", 0.0).withPosition(3, 0).getEntry();
      _flywheelIFactorEntry =
          flywheelLayout.add(_flywheelConf.name + " I Fac", 0.0).withPosition(3, 1).getEntry();
      _flywheelDFactorEntry =
          flywheelLayout.add(_flywheelConf.name + " D Fac", 0.0).withPosition(3, 2).getEntry();

      _flywheelKSEntry = flywheelLayout.add(_flywheelConf.name + " kS", _flywheelConf.feedforward.s)
                             .withPosition(4, 1)
                             .getEntry();
      _flywheelKVEntry = flywheelLayout.add(_flywheelConf.name + " kV", _flywheelConf.feedforward.v)
                             .withPosition(4, 2)
                             .getEntry();
      _flywheelKAEntry = flywheelLayout.add(_flywheelConf.name + " kA", _flywheelConf.feedforward.a)
                             .withPosition(4, 3)
                             .getEntry();

      _targetVelocity =
          flywheelLayout.add(" BSH Target Velocity", 0.0).withPosition(5, 2).getEntry();
      _flywheelVelocity = flywheelLayout.add("Velocity", 0.0).withPosition(5, 3).getEntry();
      _publishedTargetVelocity =
          flywheelLayout.add(" Published T Vel", 0.0).withPosition(5, 1).getEntry();
    }
  }
  public double motorToFlywheelRot(double motorRot) {
    return motorRot / _flywheelGearRatio;
  }

  public double flywheelRotToMotor(double rollerRot) {
    return rollerRot * _flywheelGearRatio;
  }

  public boolean isSafeToMove(
      IntakePivotPositions currentIntake,
      IntakePivotPositions desiredIntake,
      ShooterPivotPositions currentShooter,
      ShooterPivotPositions desiredShooter
  ) {
    // TODO: do
    return true;
  }

  public TalonFX getFlywheelMotor() {
    if (_isMinorSystemDisabled) {
      return new TalonFX(0);
    }
    return _flywheelMotor;
  }

  public Command setFlywheelPowerCommand(double power) {
    return new InstantCommand(() -> setFlywheelPower(power));
  }

  public Command setFlywheelRPSCommand(double rps) {
    return new InstantCommand(() -> setFlywheelControl(rps, false));
  }

  public Command setFlywheelRPSWithFFCommand(double rps) {
    return new InstantCommand(() -> {
      // setFlywheelControl(_targetVelocity.getDouble(rps), true);
      setFlywheelControl(rps, true);
    });
  }

  public Command startEndFlywheelPowerCommand(double power) {
    return (Commands.startEnd(() -> setFlywheelPower(power), () -> setFlywheelPower(0.0)));
  }

  public Command startEndFlywheelRPSCommand(double rps) {
    return Commands.startEnd(() -> setFlywheelControl(rps, false), () -> setFlywheelPower(0.0));
  }

  public Command startEndFlywheelRPSWithFFCommand(double rps) {
    return Commands.startEnd(() -> setFlywheelControl(rps, true), () -> setFlywheelPower(0.0));
  }
}
