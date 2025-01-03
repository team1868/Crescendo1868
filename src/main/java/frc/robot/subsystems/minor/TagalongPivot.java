package frc.robot.subsystems.minor;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotAltModes;
import frc.robot.parsers.PivotParser;
import frc.robot.parsers.json.utils.PivotConfJson;
import frc.robot.subsystems.Controlboard;
import frc.robot.utils.GeometricUtils;
import frc.robot.utils.MathUtils;
import frc.robot.utils.TagalongAngle;
import frc.robot.utils.TagalongMinorSystemBase;
import frc.robot.utils.TagalongMinorSystemInterface;
import frc.robot.utils.TagalongTrapezoidProfile;

public class TagalongPivot extends TagalongMinorSystemBase implements TagalongMinorSystemInterface {
  public PivotParser _pivotParser;
  public PivotConfJson _pivotConf;

  /* -------- Hardware: motors and sensors -------- */
  protected TalonFX _pivotMotor;
  public CANcoder _pivotCancoder;
  protected TalonFXConfiguration _pivotMotorConfiguration;
  protected CANcoderConfiguration _pivotCancoderConfiguration;
  protected Slot0Configs _pivotMotorSlot0;

  /* -------- Control: states and constants -------- */
  private boolean _holdPivotPosition = false;
  protected double _pivotGearRatio;
  public final double _defaultPivotLowerToleranceRot;
  public final double _defaultPivotUpperToleranceRot;
  public final double _absoluteRangeRot;
  public final double _minPositionRot;
  public final double _maxPositionRot;
  public final double _maxVelocityRPS, _maxAccelerationRPS2;
  public final double _profileTargetOffset;

  /* -------- Control: controllers and utilities -------- */
  protected ArmFeedforward _pivotFF;
  protected TagalongTrapezoidProfile.State _pivotCurState =
      new TagalongTrapezoidProfile.State(0.0, 0.0);
  protected TagalongTrapezoidProfile.State _pivotGoalState =
      new TagalongTrapezoidProfile.State(0.0, 0.0);

  protected TagalongTrapezoidProfile _pivotProfile;
  protected Timer _pivotTimer = new Timer();
  public final PositionVoltage _requestedPivotPositionVoltage = new PositionVoltage(0.0);
  protected VelocityVoltage _requestedPivotVelocityVoltage = new VelocityVoltage(0.0).withSlot(2);

  /* --- Shuffleboard Entries --- */
  protected GenericSubscriber _pivotPFactorEntry, _pivotIFactorEntry, _pivotDFactorEntry;
  protected GenericSubscriber _pivotKSEntry, _pivotKGEntry, _pivotKVEntry, _pivotKAEntry;
  protected GenericPublisher _pivotTargetPositionEntry, _pivotTargetVelocityRPSEntry;
  protected GenericPublisher _pivotCurrentPositionEntry, _pivotCurrentVelocityRPSEntry;

  // safe clamp utilities
  private double[] _values;
  private int[] _ids;

  public TagalongPivot(PivotParser pivotParser) {
    super(pivotParser);
    _pivotParser = pivotParser;

    if (_configuredMinorSystemDisable) {
      _defaultPivotLowerToleranceRot = 0.0;
      _defaultPivotUpperToleranceRot = 0.0;
      _absoluteRangeRot = 0.0;
      _minPositionRot = 0.0;
      _maxPositionRot = 0.0;
      _maxVelocityRPS = 0.0;
      _maxAccelerationRPS2 = 0.0;
      _profileTargetOffset = 0.0;
      return;
    }
    _pivotConf = _pivotParser.pivotConf;
    _pivotMotor = _pivotConf.getTalonFX();
    _pivotCancoder = _pivotConf.getCancoder();

    _pivotFF = _pivotConf.feedforward.getArmFeedforward();
    _defaultPivotLowerToleranceRot = _pivotConf.defaultTolerances.getLowerToleranceRot();
    _defaultPivotUpperToleranceRot = _pivotConf.defaultTolerances.getUpperToleranceRot();
    _minPositionRot = _pivotConf.positionalLimits.getMinRot();
    _maxPositionRot = _pivotConf.positionalLimits.getMaxRot();
    _absoluteRangeRot = _maxPositionRot - _minPositionRot;
    _maxVelocityRPS = _pivotConf.trapezoidalLimits.getLimitsRot().maxVelocity;
    _maxAccelerationRPS2 = _pivotConf.trapezoidalLimits.getLimitsRot().maxAcceleration;
    _profileTargetOffset = _pivotConf.profileOffset.getDistRotation().getRotations();

    _pivotProfile = new TagalongTrapezoidProfile(
        _pivotConf.trapezoidalLimits.getLimitsRot(), _pivotGoalState, _pivotCurState
    );
    _pivotGearRatio = _pivotConf.getMotorToMechanismRatio();
    _requestedPivotPositionVoltage.withPosition(getPivotAbsolutePositionRot());

    _pivotMotorConfiguration = _pivotConf.getPivotMotorConfiguration();
    _pivotMotorSlot0 = _pivotConf.pivotMotorControl.getSlot0();

    _pivotCancoderConfiguration = _pivotConf.getPivotCancoderConfiguration();

    double minAbs = MathUtils.cppMod(_minPositionRot, 1.0);
    double maxAbs = MathUtils.cppMod(_maxPositionRot, 1.0);
    double halfUnusedRange = (_maxPositionRot - _minPositionRot) / 2.0;
    double midUnused = maxAbs + halfUnusedRange;

    if (midUnused > 1.0) {
      _values = new double[] {midUnused - 1.0, minAbs, maxAbs, 1.0};
      _ids = new int[] {2, 0, 1, 2};
    } else if (_minPositionRot > 0.0) {
      _values = new double[] {minAbs, maxAbs, midUnused, 1.0};
      _ids = new int[] {0, 1, 2, 0};
    } else {
      _values = new double[] {maxAbs, midUnused, minAbs, 1.0};
      _ids = new int[] {1, 2, 0, 1};
    }
    if (Robot.isReal()) {
      int count = 0;
      while (!_pivotMotor.isAlive() && count <= 1000) {
        System.out.println(_pivotConf.name + " not alive " + (count++));
      }
      if (count >= 1000) {
        System.out.println(_pivotConf.name + " failed to initialize!");
      }
    }

    configCancoder();
    configPivotMotor();
  }

  public void periodic() {
    if (_isMinorSystemDisabled) {
      return;
    } else if (motorResetConfig()) {
      var goal = _pivotProfile.getGoal();
      setPivotProfile(goal.position, goal.velocity, _pivotProfile.getConstraint().maxVelocity);
    }

    // if (_pivotConf.name.equalsIgnoreCase("Intake Pivot")) {
    // System.out.println(
    //     "current: " + _pivotConf.name + getPivotMotor().getPosition().getValueAsDouble() * 360.0
    //     + " goal: " + _pivotGoalState.position * 360.0
    // );
    //   //  + "\n\n" + _pivotConf.name
    //   // + " real: " + (getPivotMotor().getPosition().getValueAsDouble() * 360.0)
    //   // );
    // }

    if (_holdPivotPosition) {
      followLastProfile();
    }
  }

  @Override
  public void onEnable() {
    if (_isMinorSystemDisabled) {
      return;
    }

    if (RobotAltModes.isPIDTuningMode && RobotAltModes.isPivotTuningMode) {
      _pivotMotorSlot0.kP = _pivotPFactorEntry.getDouble(_pivotMotorSlot0.kP);
      _pivotMotorSlot0.kI = _pivotIFactorEntry.getDouble(_pivotMotorSlot0.kI);
      _pivotMotorSlot0.kD = _pivotDFactorEntry.getDouble(_pivotMotorSlot0.kD);
      _pivotMotor.getConfigurator().apply(_pivotMotorSlot0);
    }

    if (RobotAltModes.isPivotTuningMode && _pivotConf.name.equalsIgnoreCase("Shooter Pivot")) {
      // _pivotFF = new ArmFeedforward(
      //     _pivotKSEntry.getDouble(_pivotFF.ks),
      //     _pivotKGEntry.getDouble(_pivotFF.kg),
      //     _pivotKVEntry.getDouble(_pivotFF.kv),
      //     _pivotKAEntry.getDouble(_pivotFF.ka)
      // );
      // setHoldPivotPosition(false);
      // _pivotMotor.setNeutralMode(NeutralModeValue.Coast);
      // _pivotMotor.setControl(new VelocityVoltage(0.0).withFeedForward(_pivotFF.calculate(
      //     getFFPositionRad(_pivotCancoder.getPosition().getValueAsDouble()),
      //     Units.rotationsToRadians(0.0),
      //     Units.rotationsToRadians((0.0))
      // )));
    }
  }

  @Override
  public void onDisable() {
    if (_isMinorSystemDisabled) {
      return;
    }
  }

  @Override
  public void disabledPeriodic() {
    if (_isMinorSystemDisabled) {
      return;
    }
  }

  @Override
  public void updateShuffleboard() {
    if (_isMinorSystemDisabled) {
      return;
    }

    if (RobotAltModes.isPivotTuningMode) {
      _pivotCurrentPositionEntry.setDouble(_pivotMotor.getPosition().getValueAsDouble());
      _pivotCurrentVelocityRPSEntry.setDouble(_pivotMotor.getVelocity().getValueAsDouble());

      _pivotTargetPositionEntry.setDouble(_pivotCurState.position);
      _pivotTargetVelocityRPSEntry.setDouble(_pivotCurState.velocity);
    }
  }

  public void configShuffleboard() {
    if (_isMinorSystemDisabled) {
      return;
    }

    ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning tab");
    ShuffleboardLayout pivotLayout =
        tuningTab.getLayout(_pivotConf.name + " Pivot", BuiltInLayouts.kGrid)
            .withSize(3, 4)
            .withPosition(2 * Controlboard.get()._tuningTabCounter++, 0);

    if (RobotAltModes.isPivotTuningMode) {
      _pivotCurrentPositionEntry =
          pivotLayout.add(_pivotConf.name + " Current Position", 0.0).withPosition(2, 0).getEntry();
      _pivotTargetPositionEntry =
          pivotLayout.add(_pivotConf.name + " Target Position", 0.0).withPosition(2, 1).getEntry();
      _pivotCurrentVelocityRPSEntry =
          pivotLayout.add(_pivotConf.name + " Current Velocity", 0.0).withPosition(2, 2).getEntry();
      _pivotTargetVelocityRPSEntry =
          pivotLayout.add(_pivotConf.name + " Target Velocity", 0.0).withPosition(2, 3).getEntry();

      _pivotKGEntry = pivotLayout.add(_pivotConf.name + " kG", _pivotConf.feedforward.g)
                          .withPosition(1, 0)
                          .getEntry();
      _pivotKSEntry = pivotLayout.add(_pivotConf.name + " kS", _pivotConf.feedforward.s)
                          .withPosition(1, 1)
                          .getEntry();
      _pivotKVEntry = pivotLayout.add(_pivotConf.name + " kV", _pivotConf.feedforward.v)
                          .withPosition(1, 2)
                          .getEntry();
      _pivotKAEntry = pivotLayout.add(_pivotConf.name + " kA", _pivotConf.feedforward.a)
                          .withPosition(1, 3)
                          .getEntry();
    }

    if (RobotAltModes.isPIDTuningMode && RobotAltModes.isPivotTuningMode) {
      _pivotPFactorEntry = pivotLayout.add(_pivotConf.name + " P Fac", _pivotMotorSlot0.kP)
                               .withPosition(0, 0)
                               .getEntry();
      _pivotIFactorEntry = pivotLayout.add(_pivotConf.name + " I Fac", _pivotMotorSlot0.kI)
                               .withPosition(0, 1)
                               .getEntry();
      _pivotDFactorEntry = pivotLayout.add(_pivotConf.name + " D Fac", _pivotMotorSlot0.kD)
                               .withPosition(0, 2)
                               .getEntry();
    }
  }

  public void followLastProfile() {
    if (_isMinorSystemDisabled) {
      return;
    }

    TagalongTrapezoidProfile.State nextState = _pivotProfile.calculate(_pivotTimer.get());
    _pivotMotor.setControl(
        _requestedPivotPositionVoltage.withPosition(nextState.position)
            .withFeedForward(_pivotFF.calculate(
                getFFPositionRad(_pivotCancoder.getPosition().getValueAsDouble()),
                Units.rotationsToRadians(nextState.velocity),
                Units.rotationsToRadians(
                    (nextState.velocity - _pivotCurState.velocity) / Constants.LOOP_PERIOD_S
                )
            ))
    );

    _pivotCurState = nextState;
  }

  public double getFFPositionRad(double positionRot) {
    if (_isMinorSystemDisabled) {
      return 0.0;
    }

    return Units.rotationsToRadians(positionRot)
        + _pivotConf.ffOffset.getDistRotation().getRadians();
  }

  public void setPivotPower(double power) {
    if (!_isMinorSystemDisabled) {
      setHoldPivotPosition(false);
      _pivotMotor.set(power);
    }
  }

  public double getPivotPower() {
    return _isMinorSystemDisabled ? 0.0 : _pivotMotor.get();
  }

  public void setPivotVelocity(double velocity) {
    if (!_isMinorSystemDisabled) {
      _pivotMotor.setControl(
          _requestedPivotVelocityVoltage.withVelocity(velocity).withFeedForward(_pivotFF.calculate(
              getFFPositionRad(_pivotCancoder.getPosition().getValueAsDouble()),
              Units.rotationsToRadians(velocity)
          ))
      );
    }
  }

  // Assumes the total range is < 360.0
  public double getPivotAbsolutePositionRot() {
    if (_isMinorSystemDisabled) {
      return 0.0;
    }
    return MathUtils.cppMod(_pivotMotor.getPosition().getValueAsDouble(), 1.0);
  }

  public double getPivotPosition() {
    if (_isMinorSystemDisabled) {
      return 0.0;
    }
    return _pivotMotor.getPosition().getValueAsDouble();
  }

  public void setHoldPivotPosition(boolean holdPosition) {
    _holdPivotPosition = holdPosition;
  }

  public void setPivotProfile(double goalPositionRot, double goalVelocityRPS) {
    if (_isMinorSystemDisabled) {
      return;
    }

    setPivotProfile(goalPositionRot, goalVelocityRPS, _maxVelocityRPS);
  }

  public void setPivotProfile(
      double goalPositionRot, double goalVelocityRPS, double maxVelocityRPS
  ) {
    if (_isMinorSystemDisabled) {
      return;
    }
    setHoldPivotPosition(false);

    _pivotCurState = _pivotProfile.calculate(_pivotTimer.get());
    _pivotCurState.position = _pivotMotor.getPosition().getValueAsDouble();

    _pivotGoalState.velocity = goalVelocityRPS;
    _pivotGoalState.position =
        GeometricUtils.placeInClosestRot(_pivotCurState.position, absoluteClamp(goalPositionRot))
        + _profileTargetOffset;

    _pivotProfile = new TagalongTrapezoidProfile(
        (maxVelocityRPS >= _maxVelocityRPS)
            ? _pivotConf.trapezoidalLimits.getLimitsRot()
            : new TrapezoidProfile.Constraints(maxVelocityRPS, _maxAccelerationRPS2),
        _pivotGoalState,
        _pivotCurState
    );
    _pivotTimer.restart();
  }

  public void setPivotProfile(double goalPositionRot) {
    setPivotProfile(goalPositionRot, 0.0);
  }

  public void setPivotProfile(TagalongAngle goalPositionRot) {
    setPivotProfile(goalPositionRot.getRotations(), 0.0);
  }
  public boolean isPivotProfileFinished() {
    return _isMinorSystemDisabled || _pivotProfile.isFinished(_pivotTimer.get());
  }

  private void configCancoder() {
    if (_isMinorSystemDisabled) {
      return;
    }

    _pivotCancoder.getConfigurator().apply(_pivotCancoderConfiguration);
  }

  public void configPivotMotor() {
    if (_isMinorSystemDisabled) {
      return;
    }
    _pivotMotor.getConfigurator().apply(_pivotMotorConfiguration);
  }

  public double clampPivotPosition(double target) {
    return clamp(target, _minPositionRot, _maxPositionRot);
  }

  public boolean inPivotTolerance(double lowerBound, double upperBound) {
    double position = MathUtils.cppMod(getPivotAbsolutePositionRot(), 1.0);
    return _isMinorSystemDisabled || inTolerance(position, lowerBound, upperBound)
        || inTolerance(position + 1.0, lowerBound, upperBound);
  }

  public boolean motorResetConfig() {
    if (_pivotMotor.hasResetOccurred()) {
      configPivotMotor();
      return true;
    }
    if (_pivotCancoder.hasResetOccurred()) {
      configCancoder();
      return true;
    }

    return false;
  }

  // TODO IMPLEMENT
  public boolean checkInitStatus() {
    return super.checkInitStatus();
  }

  // TODO IMPLEMENT
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

  public boolean isSafeToMove() {
    if (_isMinorSystemDisabled) {
      return true;
    }
    return true;
  }

  public TalonFX getPivotMotor() {
    if (_isMinorSystemDisabled) {
      return new TalonFX(0);
    }
    return _pivotMotor;
  }

  public double absoluteClamp(double value) {
    double abs = MathUtils.cppMod(value, 1.0);
    int i = 0;
    while (abs >= _values[i] && i < _values.length) {
      i++;
    }
    switch (_ids[i]) {
      case 2:
        return _maxPositionRot;
      case 1:
        return abs;
      case 0:
      default:
        return _minPositionRot;
    }
  }
}
