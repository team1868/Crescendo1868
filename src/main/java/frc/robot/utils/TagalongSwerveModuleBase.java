package frc.robot.utils;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotAltModes;
import frc.robot.parsers.SwerveParser;
import frc.robot.parsers.json.SwerveModuleControlJson;
import frc.robot.parsers.json.utils.swerve.SwerveModuleJson;
import frc.robot.parsers.json.utils.swerve.SwerveModuleJson.EncoderType;
import frc.robot.parsers.json.utils.swerve.SwerveModuleTypeConfJson;
import javax.swing.text.Position;
import org.littletonrobotics.junction.Logger;

// Encoderless swerve module implementation
public class TagalongSwerveModuleBase {
  public static final double alignmentToleranceDeg = 5.0;

  /* --- Module Identifiers --- */
  public final int _moduleNumber;
  public final String _moduleNumberStr;
  public final String _moduleLoggingStr;

  /* --- Sensors, motors, and hardware --- */
  protected final TalonFX _steerMotor;
  protected final TalonFX _driveMotor;
  protected final TalonFXConfiguration _steerMotorConfig;
  protected final Slot0Configs _steerSlot0Configs;
  protected final CANcoderConfiguration _swerveCancoderConfig;

  /* --- State and Physical Property variables --- */
  protected SwerveModuleState _desiredState = new SwerveModuleState();
  protected SwerveModuleState _curState = new SwerveModuleState();
  protected SwerveModulePosition _curPosition = new SwerveModulePosition();
  protected double _lastAngleRot;
  protected double _percentOutput = 0.0;
  protected double _velocity = 0.0;
  protected double _angleRot = 0.0;
  protected double _absolutePosition;
  protected final double _angleOffsetRotation;

  /* --- Control Utils --- */
  protected SimpleMotorFeedforward _feedforward;
  protected VelocityVoltage _requestedDriveVelocityVoltage = new VelocityVoltage(0.0);
  protected PositionVoltage _requestedSteerPositionVoltage = new PositionVoltage(0.0);

  /* --- Simulation resources and variables --- */
  protected TalonFXSimState _driveMotorSim;
  protected TalonFXSimState _steerMotorSim;

  protected FlywheelSim _driveWheelSim;

  protected SingleJointedArmSim _moduleAngleSim;

  /* --- Shuffleboard entries and layout --- */
  public GenericEntry _steerPFac, _steerIFac, _steerDFac;
  public GenericEntry _shuffleboardModuleEncoder;
  protected static GenericSubscriber _driveKSEntry, _driveKGEntry, _driveKVEntry, _driveKAEntry;

  // telemetry widgets
  public GenericEntry _shuffleboardModuleAngle;
  public GenericEntry _shuffleboardModuleSpeed;
  public GenericEntry _shuffleboardModuleDesiredSpeed;

  /* Threaded signal utils */
  protected final StatusSignal<Double> _drivePosition;
  protected final StatusSignal<Double> _driveVelocity;

  protected final StatusSignal<Double> _steerPosition;
  protected final StatusSignal<Double> _steerVelocity;
  protected final BaseStatusSignal[] _signals = new BaseStatusSignal[4];

  public final SwerveModuleJson _conf;
  public final SwerveModuleTypeConfJson _type;
  public final SwerveModuleControlJson _control;
  public final SwerveParser _swerveParser;
  protected final ModuleIOTalonFX _io;
  protected final ModuleIOInputsAutoLogged _inputs = new ModuleIOInputsAutoLogged();

  // TODO fix
  protected final double _theoreticalMaxWheelSpeedMPS;

  public TagalongSwerveModuleBase(
      int moduleNumber, SwerveParser swerveParser, double theoreticalMaxWheelSpeedMPS
  ) {
    _swerveParser = swerveParser;
    _conf = swerveParser.moduleConfs[moduleNumber];
    _type = swerveParser.moduleTypeConf;
    _control = swerveParser.moduleControlConf;

    _moduleNumber = moduleNumber;
    _theoreticalMaxWheelSpeedMPS = theoreticalMaxWheelSpeedMPS;

    // Derived values
    _moduleNumberStr = "M" + Integer.toString(_moduleNumber);
    _moduleLoggingStr = "Drive/Module" + _moduleNumber;

    // Convert configuration into core motors and sensors
    _feedforward = _control.driveControl.feedforward.getSimpleMotorFeedforward();

    _driveMotor = _conf.drive.getTalonFX();
    _steerMotor = _conf.getSteerMotorTalonFX();

    // Doing nothing here, but extended versions of this will do things with these
    _swerveCancoderConfig = _conf.encoderConfig.getCancoderConfig();
    _angleOffsetRotation = getEncoderOffsetRot();

    initEncoder();

    // TODO: replace with subsystem disablement control
    waitForCAN();

    // Doing nothing here, but extended versions of this will do things with this
    configEncoder();

    _steerMotorConfig = _swerveParser.getSteerConfigurationWithFeedbackConfigs(_conf);
    _steerSlot0Configs = _steerMotorConfig.Slot0;

    configSteerMotor();
    configDriveMotor();
    resetToAbsolute();

    configShuffleboard();
    simulationInit();

    _drivePosition = _driveMotor.getPosition().clone();
    _driveVelocity = _driveMotor.getVelocity().clone();

    _steerPosition = _steerMotor.getPosition().clone();
    _steerVelocity = _steerMotor.getVelocity().clone();

    _signals[0] = _drivePosition;
    _signals[1] = _driveVelocity;
    _signals[2] = _steerPosition;
    _signals[3] = _steerVelocity;

    _io = new ModuleIOTalonFX(this) {};
  }

  public void updateInputs() {
    _io.updateInputs(_inputs);
  }

  public BaseStatusSignal[] getSignals() {
    return _signals;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // TODO cleanup
    _desiredState = optimize(
        desiredState, Rotation2d.fromRotations(_steerMotor.getPosition().getValueAsDouble())
    );

    if (isOpenLoop) {
      _percentOutput = _desiredState.speedMetersPerSecond / _theoreticalMaxWheelSpeedMPS;
      _driveMotor.set(_percentOutput);
    } else {
      _velocity = MPSToRotorRPS(_desiredState.speedMetersPerSecond);
      _driveMotor.setControl(_requestedDriveVelocityVoltage.withVelocity(_velocity).withFeedForward(
          _feedforward.calculate(_desiredState.speedMetersPerSecond)
      ));
    }
    setDesiredSteer();
  }

  protected void setDesiredSteer() {
    _angleRot =
        Math.abs(_desiredState.speedMetersPerSecond) <= (_theoreticalMaxWheelSpeedMPS * 0.01)
        ? _lastAngleRot
        : _desiredState.angle.getRotations();

    // TODO: Reuse the position voltage object rather than creating a new one each time
    _steerMotor.setControl(
        _requestedSteerPositionVoltage.withPosition(_angleRot * _type.getSteerRatio())
    );
    _lastAngleRot = _angleRot;
  }

  public double getDesiredSteer() {
    return _angleRot * _type.getSteerRatio();
  }

  public double MPSToRotorRPS(double velocityMPS) {
    return velocityMPS * _type.getDriveRatio() / _type.getWheelCircumferenceM();
  }

  public double RotorRPSToMPS(double rotorRPS) {
    return rotorRPS * _type.getWheelCircumferenceM() / _type.getDriveRatio();
  }

  public SwerveModuleState getState() {
    _curState.speedMetersPerSecond = RotorRPSToMPS(_driveMotor.getVelocity().getValue());
    _curState.angle =
        Rotation2d.fromRotations(_steerMotor.getPosition().getValue() / _type.getSteerRatio());

    return _curState;
  }

  public SwerveModulePosition getPosition() {
    _curPosition.distanceMeters = _driveMotor.getRotorPosition().getValueAsDouble()
        * _type.getWheelCircumferenceM() / _type.getDriveRatio();
    _curPosition.angle = Rotation2d.fromRotations(
        _steerMotor.getPosition().getValueAsDouble() / _type.getSteerRatio()
    );

    return _curPosition;
  }

  public SwerveModulePosition getPosition(boolean refresh) {
    if (refresh) {
      /* Refresh all signals */
      _drivePosition.refresh();
      _driveVelocity.refresh();
      _steerPosition.refresh();
      _steerVelocity.refresh();
    }

    /* Now latency-compensate our signals */
    double drive_rot = BaseStatusSignal.getLatencyCompensatedValue(_drivePosition, _driveVelocity);
    double steer_rot = BaseStatusSignal.getLatencyCompensatedValue(_steerPosition, _steerVelocity);

    /*
     * Back out the drive rotations based on angle rotations due to coupling between
     * azimuth and steer
     */
    drive_rot -= steer_rot * RobotAltModes.steerDriveCouplingRatio;

    /* And push them into a SwerveModulePosition object to return */
    _curPosition.distanceMeters =
        drive_rot * _type.getWheelCircumferenceM() / _type.getDriveRatio();
    // _curPosition.distanceMeters = drive_rot / _driveRotationsPerMeter;

    // TODO check
    /* Angle is already in terms of steer rotations */
    _curPosition.angle = Rotation2d.fromRotations(steer_rot / _type.getSteerRatio());
    // only if fused
    // _curPosition.angle = Rotation2d.fromRotations(steer_rot);

    return _curPosition;
  }

  protected void waitForCAN() {
    if (Robot.isReal()) {
      int counter = 0;
      String identifier = "SWERVE " + _moduleNumberStr + " Check Init Status : ";
      while (!checkInitStatus()) {
        System.out.println(identifier + counter++);
      }
    } else {
    }
  }

  protected void configShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("SwerveModules");
    if (RobotAltModes.isVerboseMode) {
      ShuffleboardLayout layout = tab.getLayout(_moduleNumberStr, BuiltInLayouts.kGrid)
                                      .withSize(1, 4)
                                      .withPosition(_moduleNumber, 0);
      _shuffleboardModuleAngle = layout.add("angleState", 0.0).withPosition(0, 0).getEntry();
      _shuffleboardModuleSpeed = layout.add("speed", 0.0).withPosition(0, 1).getEntry();
      _shuffleboardModuleEncoder = layout.add("angleEncoder", 0.0).withPosition(0, 2).getEntry();
      _shuffleboardModuleDesiredSpeed =
          layout.add("desiredSpeed", 0.0).withPosition(0, 0).getEntry();
    }

    if (RobotAltModes.isModuleTuningMode) {
      ShuffleboardLayout layout =
          tab.getLayout(_moduleNumberStr, BuiltInLayouts.kGrid).withSize(1, 3).withPosition(4, 0);

      _driveKSEntry = layout.add("module kS", _feedforward.ks).withPosition(1, 0).getEntry();
      _driveKVEntry = layout.add("module kV", _feedforward.kv).withPosition(1, 1).getEntry();
      _driveKAEntry = layout.add("module kA", _feedforward.ka).withPosition(1, 2).getEntry();
    }
  }

  public void updateShuffleboard() {
    if (RobotAltModes.isVerboseMode) {
      _shuffleboardModuleAngle.setDouble(_curState.angle.getDegrees());
      _shuffleboardModuleDesiredSpeed.setDouble(_desiredState.speedMetersPerSecond);
      _shuffleboardModuleSpeed.setDouble(_curState.speedMetersPerSecond);
    }

    if (RobotAltModes.isModuleTuningMode) {
      _feedforward = new SimpleMotorFeedforward(
          _driveKSEntry.getDouble(_feedforward.ks),
          _driveKVEntry.getDouble(_feedforward.kv),
          _driveKAEntry.getDouble(_feedforward.ka)
      );
      // _driveMotor.setControl(new VelocityVoltage(0.0).withFeedForward(_feedforward.ks));
    }
  }

  public void alignToZero() {
    resetToAbsolute();
    _lastAngleRot = 0;
  }

  public void configPID(double ap, double ai, double ad, double p, double i, double d) {
    if (RobotAltModes.isPIDTuningMode) {
      _steerSlot0Configs.kP = ap;
      _steerSlot0Configs.kI = ai;
      _steerSlot0Configs.kD = ad;

      _control.getDriveConfiguration().Slot0.kP = p;
      _control.getDriveConfiguration().Slot0.kI = i;
      _control.getDriveConfiguration().Slot0.kD = d;

      _steerMotor.getConfigurator().apply(_steerSlot0Configs);
      _driveMotor.getConfigurator().apply(_control.getDriveConfiguration().Slot0);
    }
  }

  public void setLastAngle(SwerveModuleState desiredState) {
    SwerveModuleState goalModuleState = optimize(desiredState, getState().angle);
    _lastAngleRot = goalModuleState.angle.getRotations();
  }

  public void setLastAngle(Rotation2d angle) {
    setLastAngleRot(angle.getRotations());
  }

  public void setLastAngleRot(double angleRot) {
    _lastAngleRot = angleRot;
  }

  public boolean isAlignedTo(SwerveModuleState goalState, double toleranceDeg) {
    return Math.abs(_angleRot - goalState.angle.getRotations())
        < Units.degreesToRotations(toleranceDeg);
  }

  public boolean isAlignedTo(SwerveModuleState goalState) {
    return isAlignedTo(goalState, alignmentToleranceDeg);
  }

  protected void configSteerMotor() {
    _steerMotor.getConfigurator().apply(_steerMotorConfig);
  }

  protected void configDriveMotor() {
    _driveMotor.getConfigurator().apply(_control.getDriveConfiguration());
    _driveMotor.setPosition(0.0);
  }

  public boolean motorResetConfig() {
    boolean result = false;
    if (_steerMotor.hasResetOccurred()) {
      configSteerMotor();
      result = true;
    }
    if (_driveMotor.hasResetOccurred()) {
      configDriveMotor();
      result = true;
    }
    return result;
  }

  public boolean checkInitStatus() {
    return _steerMotor.isAlive() && _driveMotor.isAlive();
  }

  public void simulationInit() {
    if (!Robot.isReal())
      _driveMotorSim = _driveMotor.getSimState();
    _steerMotorSim = _steerMotor.getSimState();

    _driveWheelSim = new FlywheelSim(
        DCMotor.getFalcon500(1), _type.getDriveRatio(), 0.01, VecBuilder.fill(2.0 * Math.PI / 2048)
    );

    _moduleAngleSim = new SingleJointedArmSim(
        DCMotor.getFalcon500(1),
        _type.getSteerRatio(),
        0.001,
        0.0,
        -Double.MAX_VALUE,
        Double.MAX_VALUE,
        false,
        0,
        VecBuilder.fill(2.0 * Math.PI / 2048)
    );
  }

  public void periodic() {
    Logger.processInputs(_moduleLoggingStr, _inputs);
  }

  public void disabledPeriodic() {}

  public void simulationPeriodic() {
    if (RobotAltModes.isSim) {
      // Update the model motor sim _driveWheelSim, and read its angular velocity
      _driveWheelSim.setInputVoltage(_driveMotor.get() * RobotController.getBatteryVoltage());
      _driveWheelSim.update(Constants.LOOP_PERIOD_MS);

      double driveSimOmega = _driveWheelSim.getAngularVelocityRPM();
      double driveTicksPerS =
          (_driveMotor.getInverted() ? -1 : 1) * driveSimOmega * _type.getDriveRatio() / 60;

      // Update integrated sensor sim in _driveMotor
      _driveMotorSim.setRotorVelocity(driveTicksPerS);
      _driveMotorSim.setRawRotorPosition(
          _driveMotor.getRotorPosition().getValueAsDouble() + driveTicksPerS
      );

      // Update _steeringSim single-arm simulation
      _moduleAngleSim.setInputVoltage(_steerMotor.get() * RobotController.getBatteryVoltage());
      _moduleAngleSim.update(Constants.LOOP_PERIOD_MS);

      // Update integratedSensor sim in _steerMotor
      double angleSign = _steerMotor.getInverted() ? -1 : 1;
      _steerMotorSim.setRawRotorPosition(
          angleSign
          * (_moduleAngleSim.getVelocityRadPerSec() * _type.getSteerRatio() / (2 * Math.PI))
      );

      _steerMotorSim.setRawRotorPosition(
          angleSign * _moduleAngleSim.getAngleRads() * _type.getSteerRatio() / 60.0
      );
    }
  }

  public static SwerveModuleState optimize254(
      SwerveModuleState desiredState, Rotation2d currentAngle
  ) {
    // Place in closest scope
    double delta = (desiredState.angle.getDegrees() - currentAngle.getDegrees()) % 360;
    if (delta > 180.0) {
      delta += -360.0;
    } else if (delta < -180.0) {
      delta += 360.0;
    }
    double targetAngleDeg = currentAngle.getDegrees() + delta;
    double targetSpeedMPS = desiredState.speedMetersPerSecond;
    return new SwerveModuleState(targetSpeedMPS, Rotation2d.fromDegrees(targetAngleDeg));
  }

  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle
  ) {
    // Place in closest scope
    double delta = (desiredState.angle.getDegrees() - currentAngle.getDegrees()) % 360;
    if (delta > 180.0) {
      delta += -360.0;
    } else if (delta < -180.0) {
      delta += 360.0;
    }
    double targetAngleDeg = currentAngle.getDegrees() + delta;

    // Actual optimize
    double targetSpeedMPS = desiredState.speedMetersPerSecond;
    if (delta > 90.0) {
      targetSpeedMPS = -targetSpeedMPS;
      targetAngleDeg += -180.0;
    } else if (delta < -90.0) {
      targetSpeedMPS = -targetSpeedMPS;
      targetAngleDeg += 180.0;
    }
    return new SwerveModuleState(targetSpeedMPS, Rotation2d.fromDegrees(targetAngleDeg));
  }

  public TalonFX getDriveMotor() {
    return _driveMotor;
  }

  public TalonFX getSteerMotor() {
    return _steerMotor;
  }

  public EncoderType getModuleEncoderType() {
    return EncoderType.NONE;
  }

  public double getRotationAngleEncoder() {
    return _steerMotor.getPosition().getValue() / _type.getSteerRatio();
  }

  public double getDegreeAngleEncoder() {
    return getRotationAngleEncoder() * 360.0;
  }

  public Rotation2d getIOModuleAngle() {
    return Rotation2d.fromRotations(getRotationAngleEncoder());
  }

  /* ---- Only here to be overidden but still called ---- */
  protected CANcoderConfiguration getCancoderConfig() {
    return null;
  }

  protected void initEncoder() {}

  protected void configEncoder() {}

  public void resetToAbsolute() {}

  protected double getEncoderOffsetRot() {
    return 0.0;
  }

  public void onDisable() {
    _driveMotor.setNeutralMode(NeutralModeValue.Brake);
  }
  public void onEnable() {
    _driveMotor.setNeutralMode(NeutralModeValue.Coast);
  }
}
