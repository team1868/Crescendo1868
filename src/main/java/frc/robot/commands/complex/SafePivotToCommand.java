package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.enums.IntakePivotPositions;
import frc.robot.constants.enums.ShooterPivotPositions;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.GeometricUtils;
import frc.robot.utils.MathUtils;
import java.util.HashMap;

public class SafePivotToCommand extends Command {
  public static final int NUM_INTAKE_ZONES = 2;
  public static final int NUM_SHOOTER_ZONES = 3;
  public static final int NUM_ZONES = NUM_INTAKE_ZONES * NUM_SHOOTER_ZONES;

  // static and shared waypoint resources and range/id maps
  private static final double[] _corner0123 = new double[2];
  private static final double[] _corner2345 = new double[2];
  private static double[][][] _waypointSets;
  private static HashMap<Integer, double[][]> waypointMap;
  private static double[] _absoluteShooterZoneRanges, _absoluteIntakeZoneRanges;
  private static int[] _absoluteShooterZoneID, _absoluteIntakeZoneID;

  private double _intakeMax;
  private double _intakeMin;
  private double _intakeUsedRange;
  private double _intakeUnused;

  /*
   * 3 is dangerous
   *
   * 0 | 1
   * 2 [ 3
   * 4 | 5
   */

  // Current command's waypoints and configurations
  private double[][] _waypoints;
  private final boolean _holdPositionAfter;
  private double _goalVelocityRPS = 0.0;
  private double _shooterPivotVelocity;
  private double _intakePivotVelocity;

  private double _lowerIntakeBound, _upperIntakeBound;
  private double _lowerShooterBound, _upperShooterBound;

  private boolean _abort;
  private double _shooterRangeMidRot;

  /* -------- First Pivot -------- */
  private final Shooter _shooter;
  private final boolean _specifiedShooter;
  private double _desiredShooterRot;

  /* -------- Second Pivot -------- */
  private final Intake _intake;
  private final boolean _specifiedIntake;
  private double _desiredIntakeRot;

  /* -------- State counters -------- */
  private int _current = 0;
  private int _finished = 0;
  private boolean _startedFinalMove;

  public SafePivotToCommand(
      Intake intake,
      Shooter shooter,
      IntakePivotPositions desiredIntakeRot,
      ShooterPivotPositions desiredShooterRot,
      boolean holdPositionAfter
  ) {
    this(
        intake,
        shooter,
        desiredIntakeRot.getRotations(),
        desiredShooterRot.getRotations(),
        holdPositionAfter,
        intake._configuredDisable ? 0.0 : intake.getPivot()._maxVelocityRPS,
        shooter._configuredDisable ? 0.0 : shooter.getPivot()._maxVelocityRPS
    );
  }

  public SafePivotToCommand(
      Intake intake,
      Shooter shooter,
      double desiredIntakeRot,
      double desiredShooterRot,
      boolean holdPositionAfter
  ) {
    this(
        intake,
        shooter,
        desiredIntakeRot,
        desiredShooterRot,
        holdPositionAfter,
        intake._configuredDisable ? 0.0 : intake.getPivot()._maxVelocityRPS,
        shooter._configuredDisable ? 0.0 : shooter.getPivot()._maxVelocityRPS,
        true,
        true
    );
  }

  public SafePivotToCommand(
      Intake intake,
      Shooter shooter,
      IntakePivotPositions desiredIntakeRot,
      ShooterPivotPositions desiredShooterRot,
      boolean holdPositionAfter,
      double shooterPivotVelocity,
      double intakePivotVelocity

  ) {
    this(
        intake,
        shooter,
        desiredIntakeRot.getRotations(),
        desiredShooterRot.getRotations(),
        holdPositionAfter,
        shooterPivotVelocity,
        intakePivotVelocity
    );
  }

  public SafePivotToCommand(
      Intake intake,
      Shooter shooter,
      double desiredIntakeRot,
      double desiredShooterRot,
      boolean holdPositionAfter,
      double shooterPivotVelocity,
      double intakePivotVelocity
  ) {
    this(
        intake,
        shooter,
        desiredIntakeRot,
        desiredShooterRot,
        holdPositionAfter,
        shooterPivotVelocity,
        intakePivotVelocity,
        true,
        true
    );
  }

  private SafePivotToCommand(
      Intake intake,
      Shooter shooter,
      IntakePivotPositions desiredIntakeRot,
      ShooterPivotPositions desiredShooterRot,
      boolean holdPositionAfter,
      double shooterPivotVelocity,
      double intakePivotVelocity,
      boolean specifiedIntake,
      boolean specifiedShooter // TODO optimize
  ) {
    this(
        intake,
        shooter,
        desiredIntakeRot.getRotations(),
        desiredShooterRot.getRotations(),
        holdPositionAfter,
        shooterPivotVelocity,
        intakePivotVelocity,
        specifiedIntake,
        specifiedShooter
    );
  }

  private SafePivotToCommand(
      Intake intake,
      Shooter shooter,
      double desiredIntakeRot,
      double desiredShooterRot,
      boolean holdPositionAfter,
      double shooterPivotVelocity,
      double intakePivotVelocity,
      boolean specifiedIntake,
      boolean specifiedShooter // TODO optimize
  ) {
    _intake = intake;
    _shooter = shooter;
    _holdPositionAfter = holdPositionAfter;

    _specifiedIntake = specifiedIntake;
    _specifiedShooter = specifiedShooter;
    _desiredIntakeRot = desiredIntakeRot;
    _desiredShooterRot = desiredShooterRot;
    _intakePivotVelocity = intakePivotVelocity;
    _shooterPivotVelocity = shooterPivotVelocity;

    // TODO fix tolerance logic to allow for looping
    _shooterRangeMidRot = (_shooter._pivotUnsafeMinRot + _shooter._pivotUnsafeMaxRot) / 2.0;

    _intakeMax = _intake._pivotUnsafeMaxRot;
    _intakeMin = _intake._pivotUnsafeMinRot;
    _intakeUsedRange = _intakeMax - _intakeMin;
    _intakeUnused = (1.0 - _intakeUsedRange) / 2;

    // Static initialization of helpers
    initializeWaypoints();
    initializeWaypointMap();
    findShooterAbsoluteZones(shooter);

    addRequirements(intake, shooter);
  }

  public void initialize() {
    _intake.getPivot().setHoldPivotPosition(false);
    _shooter.getPivot().setHoldPivotPosition(false);
    _abort = false;

    if (!_specifiedIntake) {
      _desiredIntakeRot = _intake.getPivot().getPivotPosition();
    }
    _desiredIntakeRot =
        GeometricUtils.placeInClosestRot(_intake.getPivot().getPivotPosition(), _desiredIntakeRot);
    _lowerIntakeBound = _desiredIntakeRot - _intake.getPivot()._defaultPivotLowerToleranceRot;
    _upperIntakeBound = _desiredIntakeRot + _intake.getPivot()._defaultPivotLowerToleranceRot;

    if (!_specifiedShooter) {
      _desiredShooterRot = _shooter.getPivot().getPivotPosition();
    }
    _desiredShooterRot = GeometricUtils.placeInClosestRot(
        _shooter.getPivot().getPivotPosition(), _desiredShooterRot
    );
    _lowerShooterBound = _desiredShooterRot - _shooter.getPivot()._defaultPivotLowerToleranceRot;
    _upperShooterBound = _desiredShooterRot + _shooter.getPivot()._defaultPivotUpperToleranceRot;

    double shooterPosition = _shooter.getPivot().getPivotAbsolutePositionRot();
    _waypoints = getWaypoints(
        getZone(_intake.getPivot().getPivotAbsolutePositionRot(), shooterPosition),
        getZone(_desiredIntakeRot, _desiredShooterRot),
        shooterPosition > _shooterRangeMidRot
    );

    _current = 0;
    _finished = 0;
    _startedFinalMove = false;
  }

  @Override
  public void execute() {
    if (_abort) {
      return;
    }

    if ((_waypoints.length == 0 && _current == 0)
        || (!_startedFinalMove && _finished == _waypoints.length)) {
      // Start final movement
      _intake.getPivot().setPivotProfile(_desiredIntakeRot, _goalVelocityRPS, _intakePivotVelocity);
      _shooter.getPivot().setPivotProfile(
          _desiredShooterRot, _goalVelocityRPS, _shooterPivotVelocity
      );
      _startedFinalMove = true;
      _current++;
    } else if (_current == _finished && _current < _waypoints.length) {
      // Set next waypoint
      _intake.getPivot().setPivotProfile(_waypoints[_current][0], 0.0, _intakePivotVelocity);
      _shooter.getPivot().setPivotProfile(_waypoints[_current][1], 0.0, _shooterPivotVelocity);
      _current++;
    }

    // If current profiles are finished
    if (_intake.getPivot().isPivotProfileFinished()
        && _shooter.getPivot().isPivotProfileFinished()) {
      _finished++;
    }

    _intake.getPivot().followLastProfile();
    _shooter.getPivot().followLastProfile();
  }

  @Override
  public void end(boolean interrupted) {
    if (_abort) {
      _intake.getPivot().setPivotVelocity(0.0);
      _shooter.getPivot().setPivotVelocity(0.0);
    } else {
      _intake.getPivot().setHoldPivotPosition(_holdPositionAfter);
      _shooter.getPivot().setHoldPivotPosition(_holdPositionAfter);
    }
  }

  @Override
  public boolean isFinished() {
    return _abort
        || (_finished > _waypoints.length
            && _intake.getPivot().inTolerance(
                _intake.getPivot().getPivotPosition(), _lowerIntakeBound, _upperIntakeBound
            )
            && _shooter.getPivot().inTolerance(
                _shooter.getPivot().getPivotPosition(), _lowerShooterBound, _upperShooterBound
            ));
  }

  private int getZone(double intakePosition, double shooterPosition) {
    return getIntakeZone(intakePosition) + getShooterZone(shooterPosition);
  }

  private int getIntakeZone(double intakePosition) {
    if (_intake._configuredDisable) {
      return 0;
    }
    var absolute = MathUtils.cppMod(intakePosition, 1.0);

    if (_intakeMin < 0) {
      return (absolute > _intakeMax + _intakeUnused && absolute < _intakeMin + 1.0) ? 0 : 1;
    } else {
      return (absolute < _intakeMin || absolute > _intakeMax + _intakeUnused) ? 0 : 1;
    }
  }

  private int getShooterZone(double shooterPosition) {
    if (_shooter._configuredDisable) {
      return 0;
    }
    var absolute = MathUtils.cppMod(shooterPosition, 1.0);
    int i = 0;
    while (absolute > _absoluteShooterZoneRanges[i]) {
      i++;
    }
    return _absoluteShooterZoneID[i];
  }

  private int getMapKey(int start, int end) {
    return start * NUM_ZONES + end;
  }

  private double[][] getWaypoints(int curZone, int targetZone, boolean aboveHalfShooter) {
    int key = getMapKey(curZone, targetZone);
    _abort = targetZone == 3;

    if (curZone == 3) { // TODO -- improve this recovery to be safer
      int nearestWaypoint0123 = aboveHalfShooter ? 1 : 0;
      int need2Waypoints =
          (targetZone == 5 && aboveHalfShooter) || (targetZone == 1 && !aboveHalfShooter) ? 0 : 2;
      return _waypointSets[nearestWaypoint0123 + need2Waypoints];
    } else if (waypointMap.containsKey(key)) {
      return waypointMap.get(key);
    } else {
      return _waypointSets[4];
    }
  }

  /* ---- Compute once initialization of waypoints and a map for fast lookup ---- */
  private void initializeWaypoints() {
    if (_waypointSets == null) {
      _corner0123[0] = _intake._pivotUnsafeMinRot;
      _corner0123[1] = _shooter._pivotUnsafeMaxRot;

      _corner2345[0] = _intake._pivotUnsafeMinRot;
      _corner2345[1] = _shooter._pivotUnsafeMinRot;

      _waypointSets = new double[5][][];
      _waypointSets[0] = new double[][] {_corner2345, _corner0123};
      _waypointSets[1] = new double[][] {_corner0123, _corner2345};
      _waypointSets[2] = new double[][] {_corner2345};
      _waypointSets[3] = new double[][] {_corner0123};
      _waypointSets[4] = new double[][] {};
    }
  }

  private void initializeWaypointMap() {
    initializeWaypoints();
    if (waypointMap == null) {
      waypointMap = new HashMap<Integer, double[][]>();

      // Dual pairs
      waypointMap.put(getMapKey(5, 1), _waypointSets[0]);
      waypointMap.put(getMapKey(1, 5), _waypointSets[1]);

      // lower waypoint pairs
      waypointMap.put(getMapKey(2, 5), _waypointSets[2]);
      waypointMap.put(getMapKey(5, 2), _waypointSets[2]);

      waypointMap.put(getMapKey(0, 5), _waypointSets[2]);
      waypointMap.put(getMapKey(5, 0), _waypointSets[2]);

      // upper waypoint pairs
      waypointMap.put(getMapKey(1, 2), _waypointSets[3]);
      waypointMap.put(getMapKey(2, 1), _waypointSets[3]);

      waypointMap.put(getMapKey(1, 4), _waypointSets[3]);
      waypointMap.put(getMapKey(4, 1), _waypointSets[3]);
    }
  }

  private void findShooterAbsoluteZones(Shooter shooter) {
    if (_absoluteShooterZoneRanges == null) {
      double[] valueSet = {
          _shooter._configuredDisable ? 0.0 : _shooter.getPivot()._minPositionRot,
          _shooter._configuredDisable ? 0.0 : _shooter._pivotUnsafeMinRot,
          _shooter._configuredDisable ? 0.0 : _shooter._pivotUnsafeMaxRot,
          _shooter._configuredDisable ? 0.0 : _shooter.getPivot()._maxPositionRot};
      // todo move this to configs?
      int[] ids = {4, 4, 2, 0};

      _absoluteShooterZoneRanges = new double[6];
      _absoluteShooterZoneID = new int[6];
      fillShooterZonesAndID(valueSet, ids, _absoluteShooterZoneRanges, _absoluteShooterZoneID);
    }
  }

  private void fillShooterZonesAndID(double[] origLim, int[] origID, double[] values, int[] ids) {
    if (_shooter._configuredDisable) {
      return;
    }

    double usedRange = origLim[3] - origLim[0];
    double halfUnusedRange = (1.0 - usedRange) / 2.0;

    // find i where origLim > 0
    int i = 0;
    while (origLim[i] <= 0.0) {
      i++;
    }

    int originalI = i;
    int bottomInd = 1;
    int topInd = 4;
    // Only when min is positive can the half unused wrap around screw us over
    if (i == 0) {
      if (origLim[3] + halfUnusedRange > 1.0) {
        double val = origLim[3] + halfUnusedRange - 1.0;
        values[0] = val;
        ids[0] = 0;

        values[5] = 1.0;
        ids[5] = 0;
        i--;
      } else {
        values[0] = origLim[0];
        ids[0] = 4;

        values[5] = 1.0;
        ids[5] = 4;

        values[4] = origLim[3] + halfUnusedRange;
        ids[4] = 0;
        topInd--;
      }
    } else {
      values[0] = origLim[i];
      ids[0] = origID[i];

      values[5] = 1.0;
      ids[5] = origID[i];
    }
    i++;

    while (bottomInd <= topInd) {
      if (i == 4 && originalI != 0) {
        // overlap
        values[bottomInd] = origLim[3] + halfUnusedRange;
        ids[bottomInd] = origID[3];

        bottomInd++;
      }

      if (i > 3) {
        values[bottomInd] = Math.max(0, Math.min(1.0, origLim[i % 4] + 1.0));
        ids[bottomInd] = origID[i % 4];
      } else {
        values[bottomInd] = origLim[i];
        ids[bottomInd] = origID[i];
      }

      bottomInd++;
      i++;
    }

    _absoluteShooterZoneRanges = values;
    _absoluteShooterZoneID = ids;
  }
}
