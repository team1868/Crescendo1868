package frc.robot.commands.base;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.minor.TagalongPivot;
import frc.robot.utils.MathUtils;
import frc.robot.utils.PivotAugment;
import frc.robot.utils.TagalongAngle;
import frc.robot.utils.TagalongSubsystemBase;

public class PivotToCommand<T extends TagalongSubsystemBase & PivotAugment> extends Command {
  private final TagalongPivot _pivot;
  private double _goalPositionRot; // private final double _goalPositionRot;

  private final boolean _holdPositionAfter;
  private final double _lowerBound;
  private final double _upperBound;
  private double _maxVelocity;

  private boolean _startedMovement;
  private double _goalVelocityRPS = 0.0;

  public PivotToCommand(
      T pivot, TagalongAngle goalPosition, boolean holdPositionAfter, double maxVelocityRPS
  ) {
    this(
        pivot,
        goalPosition,
        holdPositionAfter,
        maxVelocityRPS,
        pivot._configuredDisable ? 0.0 : pivot.getPivot()._defaultPivotLowerToleranceRot,
        pivot._configuredDisable ? 0.0 : pivot.getPivot()._defaultPivotUpperToleranceRot
    );
  }

  public PivotToCommand(
      T pivot,
      TagalongAngle goalPosition,
      boolean holdPositionAfter,
      double maxVelocityRPS,
      double lowerToleranceRot,
      double upperToleranceRot
  ) {
    _pivot = pivot.getPivot();
    _goalPositionRot = goalPosition.getRotations();
    _holdPositionAfter = holdPositionAfter;
    _maxVelocity = maxVelocityRPS;
    _lowerBound = MathUtils.cppMod(_goalPositionRot, 1.0) - Math.abs(lowerToleranceRot);
    _upperBound = MathUtils.cppMod(_goalPositionRot, 1.0) + Math.abs(upperToleranceRot);

    addRequirements(pivot);
  }

  public PivotToCommand(T pivot, TagalongAngle goalPosition, boolean holdPositionAfter) {
    this(
        pivot,
        goalPosition,
        holdPositionAfter,
        pivot._configuredDisable ? 0.0 : pivot.getPivot()._maxVelocityRPS
    );
  }

  @Override
  public void initialize() {
    _pivot.setHoldPivotPosition(false);
    _startedMovement = false;
  }

  @Override
  public void execute() {
    if (!_startedMovement && _pivot.isSafeToMove()) {
      _startedMovement = true;
      _pivot.setPivotProfile(_goalPositionRot, _goalVelocityRPS, _maxVelocity);
    }

    if (_startedMovement) {
      _pivot.followLastProfile();
    }
  }

  @Override
  public void end(boolean interrupted) {
    // if (!interrupted) {
    _pivot.setHoldPivotPosition(_holdPositionAfter);
    // }
  }

  @Override
  public boolean isFinished() {
    return _pivot.isPivotProfileFinished() && _pivot.inPivotTolerance(_lowerBound, _upperBound);
  }
}
