package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.confs.BaseShooterConf;
import frc.robot.constants.enums.IntakePivotPositions;
import frc.robot.constants.enums.ShooterPivotPositions;
import frc.robot.parsers.ShooterParser;
import frc.robot.parsers.json.ShooterConfJson;
import frc.robot.utils.ShooterIOInputsAutoLogged;
import frc.robot.utils.ShooterIOTalonFX;
import org.littletonrobotics.junction.Logger;
import tagalong.math.AlgebraicUtils;
import tagalong.math.GeometricUtils;
import tagalong.subsystems.TagalongSubsystemBase;
import tagalong.subsystems.micro.Pivot;
import tagalong.subsystems.micro.Roller;
import tagalong.subsystems.micro.augments.PivotAugment;
import tagalong.subsystems.micro.augments.RollerAugment;

public class Shooter extends TagalongSubsystemBase implements PivotAugment, RollerAugment {
  public static final class ShooterConstants {
    public static final int FEEDER_ID = 0;
    public static final int FLYWHEEL_ID = 1;
  }

  public final BaseShooterConf _shooterConf;
  public final ShooterConfJson _shooterParserConf;

  private final Roller _flywheel;
  private final Pivot _pivot;
  private final Roller _roller;

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
  public Roller getRoller(int id) {
    switch (id) {
      case ShooterConstants.FLYWHEEL_ID:
        return _flywheel;
      case ShooterConstants.FEEDER_ID:
        return _roller;
    }
    return _roller;
  }

  public static class ShotConstants {
    public static final double ROLLER_IN_RPS = 40.0;
    public static final double ROLLER_SLOW_IN_RPS = 10.0;
    public static final double ROLLER_OUT_RPS = -30.0;
    public static final double FLYWHEEL_WAIT_S = 0.2; // TODO tune RYAN reduced from 0.5
    public static final double SHOT_TIMEOUT_S = 4.0; // TODO tune
    public static final double ROLLER_EJECT_RPS = -40.0;
    public static final double EJECT_WAIT_S = 1.0;
    public static final double FLYWHEEL_HOLD_RPS = -15.0;

    public static final double SUBWOOFER_SPEAKER_SPEED_RPS = 60.0;

    // SPEAKER
    public static final double FLYWHEEL_SPEAKER_SPEED_RPS = 90.0;
    // ^^ above this is tuned, below is not 2/28
    public static final double FLYWHEEL_SPEAKER_CHECK_SPEED_RPS = 20.0;
    public static final double ROLLER_SPEAKER_RPS = 90.0; // TODO tune

    // AMP
    public static final double AMP_PRE_SHOT_WAIT_S = 0.2;
    public static final double AMP_PIVOT_VELO_RPS = 1.8;
    public static final double ROLLER_AMP_PREP_RPS = -40.0; // TODO tune
    public static final double ROLLER_AMP_RPS = -90.0; // TODO tune
    public static final double FLYWHEEL_AMP_SPEED_RPS = 14.0; // TODO tune
    public static final double AMP_TIMEOUT_S = 0.25; // TODO tune
    public static final double AMP_CLEANUP_LAUNCH = 40.0;
    public static final double AMP_LAUNCH_CHECK_SPEED_RPS = 5.0;

    // AMP2
    public static final double AMP2_FLYWHEEL_RPS = 15.0;

    // TRAP
    // public static final double ROLLER_AMP_RPS = 10.0; // TODO tune
    public static final double ROLLER_TRAP_RPS = 0.0;
    public static final double TRAP_TIMEOUT_S = 0.3; // TODO tune

    public static final double FLYWHEEL_SOURCE_SPEED_RPS = -10.0;
    public static final double ROLLER_SOURCE_SPEED_RPS = -10.0;
    public static final double ROLLER_SOURCE_SPEED_2_RPS = 40.0;

    // SOTF
    public static final double NOTE_SPEAKER_INIT_MPS = 1.0;
    public static final double vSq = NOTE_SPEAKER_INIT_MPS * NOTE_SPEAKER_INIT_MPS;
    public static final double vQuartic = vSq * vSq;
    public static final double vSqG = vSq * Constants.GRAVITY_MPS2;

    public static final double FLYWHEEL_GRIEF_SPEED_RPS = 15.0;
    public static final double ROLLER_GRIEF_SPEED_RPS = 30.0;
  }

  // /* -------- Hardware: motors and sensors -------- */
  // private TalonFX _flywheelMotor;
  // private TalonFX _flywheelFollowerMotor;
  private DigitalInput _chuteBreakBeam;
  // private TalonFXConfiguration _flywheelMotorConfig,
  // _flywheelFollowerMotorConfig;
  // protected Slot0Configs _flywheelMotorSlot0 = new Slot0Configs();

  // /* -------- Control: states and constants -------- */
  // private final double _flywheelGearRatio;
  // private final VelocityVoltage _requestedFlywheelVelocityVoltage = new
  // VelocityVoltage(0.0);
  public final double _pivotUnsafeMinRot;
  public final double _pivotUnsafeMaxRot;

  /* -------- Logging: utilities and configs -------- */
  private final ShooterIOTalonFX _io;
  private final ShooterIOInputsAutoLogged _inputs = new ShooterIOInputsAutoLogged();

  /* --- Shuffleboard Entries --- */
  protected GenericSubscriber _flywheelPFactorEntry, _flywheelIFactorEntry, _flywheelDFactorEntry;
  protected GenericSubscriber _flywheelKSEntry, _flywheelKVEntry, _flywheelKAEntry;
  protected GenericSubscriber _targetVelocity;
  protected GenericPublisher _flywheelVelocity, _publishedTargetVelocity;
  protected GenericEntry _breakBeamEntry, _inChuteEntry;

  protected GenericSubscriber _ampSpeedEntry;

  public double _ampTargetSpeed = 90.0;
  protected SimpleMotorFeedforward _flywheelFF;

  public Shooter(BaseShooterConf shooterConf, String filePath) {
    this(
        shooterConf,
        filePath == null ? null : new ShooterParser(Filesystem.getDeployDirectory(), filePath)
    );
  }

  public Shooter(BaseShooterConf shooterConf, ShooterParser parser) {
    super(shooterConf);
    _flywheel = new Roller(shooterConf.flywheelConf);
    _pivot = new Pivot(shooterConf.pivotConf);
    _roller = new Roller(shooterConf.rollerConf);

    if (_flywheel._configuredMicrosystemDisable || _pivot._configuredMicrosystemDisable
        || _roller._configuredMicrosystemDisable) {
      _io = null;
      _shooterConf = null;
      _shooterParserConf = null;
      _pivotUnsafeMinRot = 0.0;
      _pivotUnsafeMaxRot = 0.0;
      return;
    }
    _shooterConf = shooterConf;
    _shooterParserConf = parser.shooterConf;

    _io = new ShooterIOTalonFX(this);
    _pivotUnsafeMinRot = _shooterConf.pivotUnsafeMin;
    _pivotUnsafeMaxRot = _shooterConf.pivotUnsafeMax;

    int counter = 0;
    while (!checkInitStatus() && counter < 100) {
      System.out.println("Waiting for Shooter");
    }

    if (counter >= 100) {
      System.out.println("failed to init Shooter");
    }

    _chuteBreakBeam = new DigitalInput(_shooterConf.breakBeamChannel);

    configShuffleboard();
  }

  public boolean isPieceInChute() {
    if (_isSubsystemDisabled) {
      return false;
    }
    // System.out.println("break beam" + !_chuteBreakBeam.get());
    return !_chuteBreakBeam.get();
  }

  @Override
  public void onEnable() {
    if (_isSubsystemDisabled) {
      return;
    }

    // if (RobotAltModes.isFlywheelTuningMode) {
    // _flywheelFF = new SimpleMotorFeedforward(
    // _flywheelKSEntry.getDouble(_pivotConf.feedforward.s),
    // _flywheelKVEntry.getDouble(_pivotConf.feedforward.v),
    // _flywheelKAEntry.getDouble(_pivotConf.feedforward.a)
    // );
    // }

    // if (RobotAltModes.isFlywheelTuningMode && RobotAltModes.isPIDTuningMode) {
    // _flywheelMotorSlot0.kP =
    // _flywheelPFactorEntry.getDouble(_flywheelMotorSlot0.kP);
    // _flywheelMotorSlot0.kI =
    // _flywheelIFactorEntry.getDouble(_flywheelMotorSlot0.kI);
    // _flywheelMotorSlot0.kD =
    // _flywheelDFactorEntry.getDouble(_flywheelMotorSlot0.kD);
    // _flywheelMotor.getConfigurator().apply(_flywheelMotorSlot0);
    // _flywheelFollowerMotor.getConfigurator().apply(_flywheelMotorSlot0);
    // }

    _flywheel.onEnable();
    _pivot.onEnable();
    _roller.onEnable();

    _flywheel.setRollerPower(0.0);
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
    _flywheel.onDisable();
    _pivot.onDisable();
    _roller.onDisable();

    _flywheel.setRollerPower(0.0);
    _pivot.setPivotPower(0.0);
    _pivot.setFollowProfile(true);
    _roller.setRollerPower(0.0);
    _pivot.getPrimaryMotor().setNeutralMode(NeutralModeValue.Brake);
    _pivot.setPivotProfile(_pivot.getPivotPosition(), 0.0);
    _pivot.getPrimaryMotor().setControl(new PositionVoltage(_pivot.getPivotPosition()));
  }

  @Override
  public void periodic() {
    // System.out.println("shooter pos" + _pivot.getPivotAbsolutePositionRot());
    // System.out.println("piece in chute" + isPieceInChute());
    // System.out.println(
    // "s " + _pivotConf.feedforward.s + "g " + _pivotConf.feedforward.g + "v "
    // + _pivotConf.feedforward.v + "a " + _pivotConf.feedforward.a + "volt "
    // + getPrimaryMotor().getMotorVoltage()
    // );
    if (_isSubsystemDisabled) {
      return;
    }

    updateShuffleboard();
    _flywheel.periodic();
    _pivot.periodic();
    _roller.periodic();

    // Logging
    _io.updateInputs(_inputs);
    Logger.processInputs("Shooter", _inputs);
  }

  public void disabledPeriodic() {
    if (_isSubsystemDisabled) {
      return;
    }
  }

  @Override
  public boolean checkInitStatus() {
    return super.checkInitStatus() && _flywheel.checkInitStatus() && _pivot.checkInitStatus()
        && _roller.checkInitStatus();
  }

  @Override
  public void simulationInit() {
    if (Robot.isReal()) {
      return;
    }
    _flywheel.simulationInit();
    _pivot.simulationInit();
    _roller.simulationInit();
  }

  @Override
  public void simulationPeriodic() {
    if (Robot.isReal()) {
      return;
    }
    _flywheel.simulationPeriodic();
    _pivot.simulationPeriodic();
    _roller.simulationPeriodic();
  }

  public boolean isOnTarget() {
    if (_isSubsystemDisabled) {
      return false;
    }
    return false;
  }

  @Override
  protected void updateShuffleboard() {
    _flywheel.updateShuffleboard();
    _pivot.updateShuffleboard();
    _roller.updateShuffleboard();
    boolean pieceInChute = isPieceInChute();
    _breakBeamEntry.setBoolean(pieceInChute);
    _inChuteEntry.setBoolean(pieceInChute);
  }

  @Override
  protected void configShuffleboard() {
    _flywheel.configShuffleboard();
    _pivot.configShuffleboard();
    _roller.configShuffleboard();
    ShuffleboardTab systemCheckTab = Shuffleboard.getTab("Systems Check");
    ShuffleboardTab feedbackTab = Shuffleboard.getTab("Feedback tab");
    boolean pieceInChute = isPieceInChute();
    _breakBeamEntry = systemCheckTab.add(_shooterConf.name + "Chute Break Beam", pieceInChute)
                          .withPosition(6, 2)
                          .withSize(2, 1)
                          .getEntry();

    _inChuteEntry =
        feedbackTab.add("In Shooter", pieceInChute).withPosition(6, 0).withSize(4, 4).getEntry();

    _ampSpeedEntry = systemCheckTab.add("rps", 0).withPosition(7, 4).getEntry();
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

  public boolean isFlywheelAtSpeakerSpeed() {
    // System.out.println(
    // "fly at speed "
    // + _flywheel.isRollerAtTargetSpeed(ShotConstants.FLYWHEEL_SPEAKER_SPEED_RPS)
    // );
    return _flywheel.isRollerAtTargetSpeed(ShotConstants.FLYWHEEL_SPEAKER_SPEED_RPS);
  }

  // return a LED speed between 0 and 1;
  public double getPercentOfSpeedLEDSpeedSpeaker() {
    return getPercentOfSpeedLEDSpeed(ShotConstants.FLYWHEEL_SPEAKER_SPEED_RPS);
  }

  public double getPercentOfSpeedLEDSpeedAmp() {
    return getPercentOfSpeedLEDSpeed(ShotConstants.ROLLER_AMP_PREP_RPS);
  }

  public double getPercentOfSpeedLEDSpeedClimb() {
    return getPercentOfSpeedLEDSpeed(ShotConstants.ROLLER_OUT_RPS);
  }

  public double getPercentOfSpeedLEDSpeed(double target) {
    return 1.0
        - (Math.abs(target - _flywheel.getPrimaryMotor().getVelocity().getValueAsDouble()) / target
        );
  }

  // public void setPivotPositionFromTarget(Translation2d currentRobot,
  // Translation3d target) {
  // _pivot.getPrimaryMotor().setControl(_pivot._requestedPositionVoltage.withPosition(
  // Units.radiansToRotations(getLaunchAngleFromTargetPhysicsRad(currentRobot,
  // target))));
  // }

  // public void setPivotPositionFromTargets(
  // Translation2d currentRobot, Translation3d target1, Translation3d target2) {
  // _pivot.getPrimaryMotor().setControl(
  // _pivot._requestedPositionVoltage.withPosition(Units.degreesToRotations(
  // (getLaunchAngleFromTargetPhysicsRad(currentRobot, target1)
  // + getLaunchAngleFromTargetPhysicsRad(currentRobot, target2))
  // / 2.0)));
  // }

  public void setPivotProfileFromTargetPhysics(Translation2d currentRobot, Translation3d target) {
    _pivot.setPivotProfile(
        Units.radiansToRotations(getLaunchAngleFromTargetPhysicsRad(currentRobot, target)), 0.0
    );
  }

  public void setPivotProfileFromTargetLinearized(
      Translation2d currentRobot, Translation2d target
  ) {
    setPivotProfileFromTargetLinearized(currentRobot.getDistance(target));
  }

  public double setPivotProfileFromTargetLinearized(double distance) {
    var target = getTargetLaunchAngleFromTargetLinearizedRot(distance);
    _pivot.setPivotProfile(target, 0.0);
    return target;
  }

  public void setPivotProfileFromTargetPolynomial(
      Translation2d currentRobot, Translation2d target
  ) {
    setPivotProfileFromTargetPolynomial(currentRobot.getDistance(target));
  }

  public double setPivotProfileFromTargetPolynomial(double distance) {
    var target = getTargetLaunchAngleFromTargetPolynomialRot(distance);
    _pivot.setPivotProfile(target, 0.0);
    return target;
  }

  public void setPivotProfileFromTargetsPhysics(
      Translation2d currentRobot, Translation3d target1, Translation3d target2
  ) {
    _pivot.setPivotProfile(
        Units.radiansToRotations(
            (getLaunchAngleFromTargetPhysicsRad(currentRobot, target1)
             + getLaunchAngleFromTargetPhysicsRad(currentRobot, target2))
            / 2.0
        ),
        0.0
    );
  }

  public double getMinLaunchAngleFromTargetLinearizedRot(double distFromTargetM) {
    return Units.degreesToRotations(
        _shooterParserConf.minPivotLinearized.getDegOutput(distFromTargetM)
    );
  }

  public double getMaxLaunchAngleFromTargetLinearizedRot(double distFromTargetM) {
    return Units.degreesToRotations(
        _shooterParserConf.maxPivotLinearized.getDegOutput(distFromTargetM)
    );
  }

  public double getTargetLaunchAngleFromTargetLinearizedRot(double distFromTargetM) {
    return Units.degreesToRotations(
        _shooterParserConf.targetPivotLinearized.getDegOutput(distFromTargetM)
    );
  }

  public double get(double distFromTargetM) {
    return Units.degreesToRotations(
        _shooterParserConf.targetPivotLinearized.getDegOutput(distFromTargetM)
    );
  }

  public double getMinLaunchAngleFromTargetPolynomialRot(double distFromTargetM) {
    if (_isSubsystemDisabled) {
      return -1.0;
    }
    return _shooterParserConf.minPivotQuadratic.getRotOutput(distFromTargetM);
  }

  public double getMaxLaunchAngleFromTargetPolynomialRot(double distFromTargetM) {
    if (_isSubsystemDisabled) {
      return -1.0;
    }
    return _shooterParserConf.maxPivotQuadratic.getRotOutput(distFromTargetM);
  }

  public double getTargetLaunchAngleFromTargetPolynomialRot(double distFromTargetM) {
    if (_isSubsystemDisabled) {
      return -1.0;
    }
    return _shooterParserConf.targetPivotQuadratic.getRotOutput(distFromTargetM);
  }

  public double getLaunchAngleFromTargetPhysicsRad(
      Translation2d currentRobot, Translation3d target
  ) {
    double x = currentRobot.getDistance(target.toTranslation2d());
    double h = target.getZ();

    // ignore drivetrain movement
    // double v = ShotConstants.NOTE_SPEAKER_INIT_MPS;
    // formula:
    // https://en.wikipedia.org/wiki/Projectile_motion#Angle_%CE%B8_required_to_hit_coordinate_(x,_y)
    double gx = Constants.GRAVITY_MPS2 * x;
    // Optimization of the following
    // Math.atan((vSq + Math.sqrt(Math.pow(v, 4) - g * (g * x * x + 2 * h * vSq))) /
    // (g * x));
    return Math.atan(
        (ShotConstants.vSq
         + Math.sqrt(ShotConstants.vQuartic - (gx * gx + 2 * h * ShotConstants.vSqG)))
        / (gx)
    );
    // clamp(, _minPositionRot, _maxPositionRot);
  }

  public void finishShot() {
    _flywheel.setRollerPower(0.0);
    _roller.setRollerPower(0.0);
  }

  public boolean isOnSOTFTarget(double robotDistFromTargetM) {
    // System.out.println(
    // "cur" + _pivot.getPivotAbsolutePositionRot() + "min"
    // + getMinLaunchAngleFromTargetPolynomialRot(robotDistFromTargetM) + "max"
    // + getMaxLaunchAngleFromTargetPolynomialRot(robotDistFromTargetM)
    // );
    double pivot = _pivot.getPivotPosition();
    // System.out.print(
    // "pivot pos " + _pivot.getPivotPosition() * 360.0 + " pivot min pos "
    // + AlgebraicUtils.placeInScopeRot(
    // pivot, getMinLaunchAngleFromTargetPolynomialRot(robotDistFromTargetM)
    // ) * 360.0
    // + " pivot max pos "
    // + AlgebraicUtils.placeInScopeRot(
    // pivot, getMaxLaunchAngleFromTargetPolynomialRot(robotDistFromTargetM)
    // )
    // );
    double min = AlgebraicUtils.placeInScopeRot(
        pivot, getMinLaunchAngleFromTargetPolynomialRot(robotDistFromTargetM)
    );
    double max = AlgebraicUtils.placeInScopeRot(
        pivot, getMaxLaunchAngleFromTargetPolynomialRot(robotDistFromTargetM)
    );

    return pivot >= min && pivot <= max;
  }

  public boolean isOnSOTFTargetWithMinRange(double robotDistFromTargetM, double minRangeRot) {
    double pivot = _pivot.getPivotPosition();
    // this has a weird edge case, but is fine given our range of motion
    double mid = AlgebraicUtils.placeInScopeRot(
        pivot, getTargetLaunchAngleFromTargetLinearizedRot(robotDistFromTargetM)
    );
    double min = AlgebraicUtils.placeInScopeRot(
        pivot, getMinLaunchAngleFromTargetLinearizedRot(robotDistFromTargetM)
    );
    double max = AlgebraicUtils.placeInScopeRot(
        pivot, getMaxLaunchAngleFromTargetLinearizedRot(robotDistFromTargetM)
    );
    // System.out.println(
    // "pivot " + Units.rotationsToDegrees(pivot) + " mid " +
    // Units.rotationsToDegrees(mid)
    // + " min " + Units.rotationsToDegrees(min) + " max " +
    // Units.rotationsToDegrees(max)
    // + "robot dist" + robotDistFromTargetM
    // );
    return pivot >= Math.min(min, mid - minRangeRot) && pivot <= Math.max(max, mid + minRangeRot);
  }

  public void setAmpLaunchVelocity(double distFromTargetM) {
    _ampTargetSpeed = _shooterParserConf.launchRPSLinearized.getRPSOutput(distFromTargetM);
    getRoller(ShooterConstants.FLYWHEEL_ID).setRollerVelocity(_ampTargetSpeed, true);
  }

  public boolean isAtAmpLaunchSpeed() {
    return Math.abs(getRoller(ShooterConstants.FLYWHEEL_ID).getRollerVelocity() - _ampTargetSpeed)
        <= ShotConstants.AMP_LAUNCH_CHECK_SPEED_RPS;
  }
}
