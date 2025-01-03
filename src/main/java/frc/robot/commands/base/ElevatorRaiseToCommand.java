package frc.robot.commands.base;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.minor.TagalongElevator;
import frc.robot.utils.ElevatorAugment;
import frc.robot.utils.TagalongHeight;
import frc.robot.utils.TagalongSubsystemBase;

public class ElevatorRaiseToCommand<T extends TagalongSubsystemBase & ElevatorAugment>
    extends Command {
  private final TagalongElevator _elevator;
  private final double _lowerBound;
  private final double _upperBound;
  private final double _goalHeightM;
  private boolean _holdPositionAfter;
  private boolean _startedMovement;
  private double _maxVelocityMPS;

  public ElevatorRaiseToCommand(T elevator, TagalongHeight goalPosition) {
    this(elevator, goalPosition, true);
  }

  public ElevatorRaiseToCommand(int id, T elevator, TagalongHeight goalPosition) {
    this(id, elevator, goalPosition, true);
  }

  public ElevatorRaiseToCommand(T elevator, TagalongHeight goalPosition, double maxVelocityMPS) {
    this(
        elevator,
        goalPosition,
        true,
        elevator.getElevator()._defaultElevatorLowerToleranceM,
        elevator.getElevator()._defaultElevatorUpperToleranceM,
        maxVelocityMPS
    );
  }

  public ElevatorRaiseToCommand(
      int id, T elevator, TagalongHeight goalPosition, double maxVelocityMPS
  ) {
    this(
        id,
        elevator,
        goalPosition,
        true,
        elevator.getElevator(id)._defaultElevatorLowerToleranceM,
        elevator.getElevator(id)._defaultElevatorUpperToleranceM,
        maxVelocityMPS
    );
  }

  public ElevatorRaiseToCommand(
      T elevator, TagalongHeight goalPosition, boolean holdPositionAfter
  ) {
    this(
        elevator,
        goalPosition,
        holdPositionAfter,
        elevator.getElevator()._defaultElevatorLowerToleranceM,
        elevator.getElevator()._defaultElevatorUpperToleranceM
    );
  }

  public ElevatorRaiseToCommand(
      int id, T elevator, TagalongHeight goalPosition, boolean holdPositionAfter
  ) {
    this(
        id,
        elevator,
        goalPosition,
        holdPositionAfter,
        elevator.getElevator(id)._defaultElevatorLowerToleranceM,
        elevator.getElevator(id)._defaultElevatorUpperToleranceM
    );
  }

  public ElevatorRaiseToCommand(
      T elevator, TagalongHeight goalPosition, boolean holdPositionAfter, double tolerance
  ) {
    this(elevator, goalPosition, holdPositionAfter, tolerance, tolerance);
  }

  public ElevatorRaiseToCommand(
      int id, T elevator, TagalongHeight goalPosition, boolean holdPositionAfter, double tolerance
  ) {
    this(id, elevator, goalPosition, holdPositionAfter, tolerance, tolerance);
  }

  public ElevatorRaiseToCommand(
      T elevator,
      TagalongHeight goalPosition,
      boolean holdPositionAfter,
      double lowerTolerance,
      double upperTolerance
  ) {
    this(
        elevator,
        goalPosition,
        holdPositionAfter,
        lowerTolerance,
        upperTolerance,
        elevator.getElevator()._maxVelocityMPS
    );
  }

  public ElevatorRaiseToCommand(
      int id,
      T elevator,
      TagalongHeight goalPosition,
      boolean holdPositionAfter,
      double lowerTolerance,
      double upperTolerance
  ) {
    this(
        id,
        elevator,
        goalPosition,
        holdPositionAfter,
        lowerTolerance,
        upperTolerance,
        elevator.getElevator(id)._maxVelocityMPS
    );
  }

  public ElevatorRaiseToCommand(
      T elevator,
      TagalongHeight goalPosition,
      boolean holdPositionAfter,
      double lowerTolerance,
      double upperTolerance,
      double maxVelocityMPS
  ) {
    _elevator = elevator.getElevator();
    _goalHeightM = goalPosition.getHeightM();
    _lowerBound = _goalHeightM - Math.abs(lowerTolerance);
    _upperBound = _goalHeightM + Math.abs(upperTolerance);
    _holdPositionAfter = holdPositionAfter;
    _maxVelocityMPS = maxVelocityMPS;

    addRequirements(elevator);
  }

  public ElevatorRaiseToCommand(
      int id,
      T elevator,
      TagalongHeight goalPosition,
      boolean holdPositionAfter,
      double lowerTolerance,
      double upperTolerance,
      double maxVelocityMPS
  ) {
    _elevator = elevator.getElevator(id);
    _goalHeightM = goalPosition.getHeightM();
    _lowerBound = _goalHeightM - Math.abs(lowerTolerance);
    _upperBound = _goalHeightM + Math.abs(upperTolerance);
    _holdPositionAfter = holdPositionAfter;
    _maxVelocityMPS = maxVelocityMPS;

    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    _startedMovement = false;
  }

  @Override
  public void execute() {
    // if elevator has not started moving, check for legal states
    if (!_startedMovement) {
      if (_elevator.isSafeToMove()) {
        _elevator.setElevatorProfile(_goalHeightM, 0.0, _maxVelocityMPS);
        _startedMovement = true;
      }
    } else {
      _elevator.followLastProfile();
    }
  }

  @Override
  public void end(boolean interrupted) {
    _elevator.setHoldElevatorPosition(_holdPositionAfter);
    double height = _elevator.getElevatorHeightM();
    // Emergency shutdown
    if (height < _elevator._elevatorMinHeightM || height > _elevator._elevatorMaxHeightM) {
      _elevator.setElevatorPower(0.0);
    }
  }

  @Override
  public boolean isFinished() {
    return (
        _elevator.isProfileFinished() && _elevator.elevatorInTolerance(_lowerBound, _upperBound)
    );
  }
}
