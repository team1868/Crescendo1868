package frc.robot.subsystems.minor;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotAltModes;
import frc.robot.parsers.ElevatorParser;
import frc.robot.parsers.json.utils.ElevatorConfJson;
import frc.robot.subsystems.Controlboard;
import frc.robot.utils.TagalongMinorSystemBase;
import frc.robot.utils.TagalongMinorSystemInterface;
import frc.robot.utils.TagalongTrapezoidProfile;

public class TagalongElevator
    extends TagalongMinorSystemBase implements TagalongMinorSystemInterface {
  protected final ElevatorParser _elevatorParser;
  protected final ElevatorConfJson _elevatorConf;

  /* -------- Hardware: motors and sensors -------- */
  protected final TalonFX _elevatorMotor;
  protected TalonFXConfiguration _elevatorMotorConf;
  protected Slot0Configs _elevatorMotorSlot0;

  /* -------- Control: states and constants -------- */
  protected boolean _holdPosition;
  protected final double _gearRatio;
  public final double _elevatorMinHeightM, _elevatorMaxHeightM;
  public final double _maxVelocityMPS, _maxAccelerationMPS2;
  public final double _defaultElevatorLowerToleranceM, _defaultElevatorUpperToleranceM;

  /* -------- Control: controllers and utilities -------- */
  protected ElevatorFeedforward _elevatorFF;
  protected final TagalongTrapezoidProfile.State _goalState =
      new TagalongTrapezoidProfile.State(0.0, 0.0);
  protected TagalongTrapezoidProfile.State _curState = new TagalongTrapezoidProfile.State(0.0, 0.0);
  protected PositionVoltage _requestedElevatorPositionVoltage = new PositionVoltage(0.0);

  protected TagalongTrapezoidProfile _elevatorProfile;
  protected Timer _timer = new Timer();

  /* --- Shuffleboard Entries --- */
  protected GenericSubscriber _elevatorPFactorEntry, _elevatorIFactorEntry, _elevatorDFactorEntry;
  protected GenericSubscriber _elevatorKSEntry, _elevatorKGEntry, _elevatorKVEntry,
      _elevatorKAEntry;
  protected GenericPublisher _elevatorCurrentPositionMEntry, _elevatorTargetPositionMEntry;
  protected GenericPublisher _elevatorCurrentVelocityMPSEntry, _elevatorTargetVelocityMPSEntry;

  public TagalongElevator(ElevatorParser parser) {
    super(parser);
    _elevatorParser = parser;

    if (_configuredMinorSystemDisable) {
      _elevatorConf = null;
      _gearRatio = 1.0;
      _elevatorMinHeightM = 0.0;
      _elevatorMaxHeightM = 0.0;
      _maxVelocityMPS = 0.0;
      _maxAccelerationMPS2 = 0.0;
      _defaultElevatorLowerToleranceM = 0.0;
      _defaultElevatorUpperToleranceM = 0.0;
      _elevatorMotor = null;
      return;
    }

    _elevatorConf = _elevatorParser.elevatorConf;

    _elevatorMotor = _elevatorConf.motor.getTalonFX();
    if (Robot.isReal()) {
      int counter = 0;
      while (!checkInitStatus() && counter <= 1000) {
        System.out.println("CLIMBER ELEVATOR Check Init Status : " + counter++);
      }
      if (counter >= 1000) {
        System.out.println(_elevatorConf.name + " failed to initialize!");
      }
    }

    _elevatorFF = _elevatorConf.feedforward.getElevatorFeedforward();
    _elevatorProfile = new TagalongTrapezoidProfile(
        _elevatorConf.trapezoidalLimits.getLimitsM(), _goalState, _curState
    );
    _gearRatio = _elevatorConf.getGearRatio();
    _elevatorMaxHeightM = _elevatorConf.positionalLimits.getMaxM();
    _elevatorMinHeightM = _elevatorConf.positionalLimits.getMinM();
    _maxVelocityMPS = _elevatorConf.trapezoidalLimits.velocity;
    _maxAccelerationMPS2 = _elevatorConf.trapezoidalLimits.acceleration;
    _defaultElevatorUpperToleranceM = _elevatorConf.defaultTolerances.getLowerToleranceM();
    _defaultElevatorLowerToleranceM = _elevatorConf.defaultTolerances.getUpperToleranceM();

    _elevatorMotorConf = _elevatorConf.motorControl.getFullMotorConfiguration();
    _elevatorMotorSlot0 = _elevatorMotorConf.Slot0;

    configElevatorMotor();
  }

  public void followLastProfile() {
    if (_isMinorSystemDisabled) {
      return;
    }

    TagalongTrapezoidProfile.State nextState = _elevatorProfile.calculate(_timer.get());
    double desiredAccelMPS2 = (nextState.velocity - _curState.velocity) / Constants.LOOP_PERIOD_S;

    // TODO: Reuse the position voltage object rather than creating a new one each time
    _elevatorMotor.setControl(
        _requestedElevatorPositionVoltage.withPosition(metersToMotor(nextState.position))
            .withFeedForward(_elevatorFF.calculate(nextState.velocity, desiredAccelMPS2))
    );
    _curState = nextState;
  }

  public void setElevatorProfile(double goalPositionM, double goalVelocityMPS) {
    if (_isMinorSystemDisabled) {
      return;
    }

    setElevatorProfile(goalPositionM, goalVelocityMPS, _maxVelocityMPS);
  }

  public void setElevatorProfile(
      double goalPositionM, double goalVelocityMPS, double maxVelocityMPS
  ) {
    if (_isMinorSystemDisabled) {
      return;
    }

    setHoldElevatorPosition(false);
    _goalState.position = clamp(
        goalPositionM,
        _elevatorConf.positionalLimits.getMinM(),
        _elevatorConf.positionalLimits.getMaxM()
    );
    _goalState.velocity = goalVelocityMPS;

    _curState = _elevatorProfile.calculate(_timer.get());
    _curState.position = getElevatorHeightM();

    _elevatorProfile = new TagalongTrapezoidProfile(
        (maxVelocityMPS >= _maxVelocityMPS)
            ? _elevatorConf.trapezoidalLimits.getLimitsM()
            : new TrapezoidProfile.Constraints(maxVelocityMPS, _maxAccelerationMPS2),
        _goalState,
        _curState
    );

    _timer.restart();
  }

  public boolean isProfileFinished() {
    return _isMinorSystemDisabled || _elevatorProfile.isFinished(_timer.get());
  }

  public void setHoldElevatorPosition(boolean hold) {
    if (_isMinorSystemDisabled) {
      return;
    }
    _holdPosition = hold;
  }

  public void setElevatorPower(double power) {
    if (_isMinorSystemDisabled) {
      return;
    }
    _elevatorMotor.set(power);
  }

  public double getElevatorPower() {
    return _isMinorSystemDisabled ? 0.0 : _elevatorMotor.get();
  }

  public double getElevatorPosition() {
    return _isMinorSystemDisabled ? 0.0 : _elevatorMotor.getPosition().getValueAsDouble();
  }

  public double getElevatorHeightM() {
    if (_isMinorSystemDisabled) {
      return 0.0;
    }
    return motorToMeters(getElevatorPosition());
  }

  public double getElevatorVelocityMPS() {
    if (_isMinorSystemDisabled) {
      return 0.0;
    }
    return motorToMeters(_elevatorMotor.getVelocity().getValueAsDouble());
  }

  public StatusCode configElevatorMotor() {
    if (_isMinorSystemDisabled) {
      return StatusCode.OK;
    }
    var status = _elevatorMotor.getConfigurator().apply(_elevatorMotorConf);
    _elevatorMotor.setPosition(0.0);
    return status;
  }

  // TODO fix the default behavior
  public double clampElevatorPosition(double target) {
    if (_isMinorSystemDisabled) {
      return 0.0;
    }
    return clamp(
        target, _elevatorConf.positionalLimits.getMinM(), _elevatorConf.positionalLimits.getMaxM()
    );
  }

  public TalonFX getElevatorMotor() {
    if (_isMinorSystemDisabled) {
      return new TalonFX(0); // TODO is this logic ok
    }
    return _elevatorMotor;
  }

  @Override
  public void onEnable() {
    if (_isMinorSystemDisabled) {
      return;
    }
    _elevatorMotor.setNeutralMode(_elevatorMotorConf.MotorOutput.NeutralMode);

    if ((RobotAltModes.isPIDTuningMode || RobotAltModes.isElevatorTuningMode)
        && _elevatorConf.name.equalsIgnoreCase(RobotAltModes.currentMicrosystem)) {
      _elevatorMotorSlot0.kP = _elevatorPFactorEntry.getDouble(_elevatorMotorSlot0.kP);
      _elevatorMotorSlot0.kI = _elevatorIFactorEntry.getDouble(_elevatorMotorSlot0.kI);
      _elevatorMotorSlot0.kD = _elevatorDFactorEntry.getDouble(_elevatorMotorSlot0.kD);
      _elevatorMotor.getConfigurator().apply(_elevatorMotorSlot0);
    }

    if (RobotAltModes.isElevatorTuningMode
        && _elevatorConf.name.equalsIgnoreCase(RobotAltModes.currentMicrosystem)) {
      _elevatorFF = new ElevatorFeedforward(
          _elevatorKSEntry.getDouble(_elevatorFF.ks),
          _elevatorKGEntry.getDouble(_elevatorFF.kg),
          _elevatorKVEntry.getDouble(_elevatorFF.kv),
          _elevatorKAEntry.getDouble(_elevatorFF.ka)
      );
      _elevatorMotor.setControl(_requestedElevatorPositionVoltage.withFeedForward(_elevatorFF.ks));
    }
  }

  @Override
  public void onDisable() {
    if (_isMinorSystemDisabled) {
      return;
    }
  }

  @Override
  public void periodic() {
    if (_isMinorSystemDisabled) {
      return;
    } else if (motorResetConfig()) {
      // should probably zero here, but turning off elevator motor is probably safest
      setHoldElevatorPosition(false);
      _elevatorMotor.set(0.0);
    }
    if (_holdPosition) {
      followLastProfile();
    }
  }

  // TODO IMPLEMENT
  @Override
  public boolean motorResetConfig() {
    if (_elevatorMotor.hasResetOccurred()) {
      configElevatorMotor();
      return true;
    }

    return false;
  }

  // TODO IMPLEMENT
  @Override
  public boolean checkInitStatus() {
    return _elevatorMotor.isAlive();
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
    if (_isMinorSystemDisabled) {
      return;
    }
    if (RobotAltModes.isElevatorTuningMode
        && _elevatorConf.name.equalsIgnoreCase(RobotAltModes.currentMicrosystem)) {
      _elevatorCurrentPositionMEntry.setDouble(getElevatorHeightM());
      _elevatorTargetPositionMEntry.setDouble(_curState.position);
      _elevatorCurrentVelocityMPSEntry.setDouble(getElevatorVelocityMPS());
      _elevatorTargetVelocityMPSEntry.setDouble(_curState.velocity);
    }
  }

  public void configShuffleboard() {
    if (_isMinorSystemDisabled) {
      return;
    }
    ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning tab");
    ShuffleboardLayout elevatorLayout =
        tuningTab.getLayout(_elevatorConf.name, BuiltInLayouts.kGrid)
            .withSize(1, 4)
            .withPosition(2 * Controlboard.get()._tuningTabCounter++, 0);
    if (RobotAltModes.isElevatorTuningMode
        && _elevatorConf.name.equalsIgnoreCase(RobotAltModes.currentMicrosystem)) {
      _elevatorCurrentPositionMEntry =
          elevatorLayout.add(_elevatorConf.name + " Current Position", 0.0)
              .withPosition(0, 0)
              .getEntry();
      _elevatorTargetPositionMEntry =
          elevatorLayout.add(_elevatorConf.name + " Target Position", 0.0)
              .withPosition(0, 1)
              .getEntry();
      _elevatorCurrentVelocityMPSEntry =
          elevatorLayout.add(_elevatorConf.name + " Current Velocity", 0.0)
              .withPosition(0, 2)
              .getEntry();
      _elevatorTargetVelocityMPSEntry =
          elevatorLayout.add(_elevatorConf.name + " Target Velocity", 0.0)
              .withPosition(0, 3)
              .getEntry();

      _elevatorKGEntry = elevatorLayout.add(_elevatorConf.name + " kG", _elevatorConf.feedforward.g)
                             .withPosition(2, 0)
                             .getEntry();
      _elevatorKSEntry = elevatorLayout.add(_elevatorConf.name + " kS", _elevatorConf.feedforward.s)
                             .withPosition(2, 1)
                             .getEntry();
      _elevatorKVEntry = elevatorLayout.add(_elevatorConf.name + " kV", _elevatorConf.feedforward.v)
                             .withPosition(2, 2)
                             .getEntry();
      _elevatorKAEntry = elevatorLayout.add(_elevatorConf.name + " kA", _elevatorConf.feedforward.a)
                             .withPosition(2, 3)
                             .getEntry();
    }
    if ((RobotAltModes.isPIDTuningMode || RobotAltModes.isElevatorTuningMode)
        && _elevatorConf.name.equalsIgnoreCase(RobotAltModes.currentMicrosystem)) {
      _elevatorPFactorEntry =
          elevatorLayout.add(_elevatorConf.name + " Height P Fac", _elevatorMotorSlot0.kP)
              .withPosition(1, 0)
              .getEntry();
      _elevatorIFactorEntry =
          elevatorLayout.add(_elevatorConf.name + " Height I Fac", _elevatorMotorSlot0.kI)
              .withPosition(1, 1)
              .getEntry();
      _elevatorDFactorEntry =
          elevatorLayout.add(_elevatorConf.name + " Height D Fac", _elevatorMotorSlot0.kD)
              .withPosition(1, 2)
              .getEntry();
    }
  }

  @Override
  public void disabledPeriodic() {
    if (_isMinorSystemDisabled) {
      return;
    }
  }

  public double motorToMeters(double rotation) {
    return _isMinorSystemDisabled ? 0.0
                                  : (rotation * _elevatorConf.getDrumCircumference()) / _gearRatio;
  }

  public double metersToMotor(double meter) {
    return _isMinorSystemDisabled ? 0.0
                                  : (meter / _elevatorConf.getDrumCircumference()) * _gearRatio;
  }

  public boolean isSafeToMove() {
    return true;
  }

  public boolean elevatorInTolerance(double lowerBound, double upperBound) {
    if (_isMinorSystemDisabled) {
      return true;
    }
    return inTolerance(getElevatorHeightM(), lowerBound, upperBound);
  }

  public void setElevatorHeight(double height) {
    if (_isMinorSystemDisabled) {
      return;
    }
    _elevatorMotor.setPosition(metersToMotor(height));
  }

  public double getPercentOfSpeedLEDSpeed() {
    return getPercentOfSpeedLEDSpeed(_goalState.position);
  }
  public double getPercentOfSpeedLEDSpeed(double goalHeight) {
    return 1.0 - (Math.abs(goalHeight - getElevatorHeightM())) / goalHeight;
  }
}
