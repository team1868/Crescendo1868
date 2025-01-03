package frc.robot.commands.base;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.minor.TagalongRoller;
import frc.robot.utils.RollerAugment;
import frc.robot.utils.TagalongAngle;

public class RollerRotateXCommand<T extends Subsystem & RollerAugment> extends Command {
  private final TagalongRoller _roller;
  private final double _relativeMovementRot; // desired
  private final double _lowerToleranceRot;
  private final double _upperToleranceRot;
  private final boolean _requireInTolerance;
  private boolean _startedMovement;
  private double _startPosition;
  private double _goalPosition;
  private double _lowerBound;
  private double _upperBound;

  public RollerRotateXCommand(T roller, TagalongAngle relativeMovement) {
    this(roller, relativeMovement.getRotations());
  }

  public RollerRotateXCommand(T roller, double relativeMovementRot) {
    this(
        roller,
        relativeMovementRot,
        roller.getRoller()._defaultRollerLowerToleranceRot,
        roller.getRoller()._defaultRollerUpperToleranceRot
    );
  }

  public RollerRotateXCommand(
      T roller, TagalongAngle relativeMovement, double lowerToleranceRot, double upperToleranceRot
  ) {
    this(roller, relativeMovement.getRotations(), lowerToleranceRot, upperToleranceRot, false);
  }

  public RollerRotateXCommand(
      T roller,
      TagalongAngle relativeMovement,
      double lowerToleranceRot,
      double upperToleranceRot,
      boolean requireInTolerance
  ) {
    this(
        roller,
        relativeMovement.getRotations(),
        lowerToleranceRot,
        upperToleranceRot,
        requireInTolerance
    );
  }

  public RollerRotateXCommand(
      T roller, double relativeMovementRot, double lowerToleranceRot, double upperToleranceRot
  ) {
    this(roller, relativeMovementRot, lowerToleranceRot, upperToleranceRot, false);
  }

  public RollerRotateXCommand(
      T roller,
      double relativeMovementRot,
      double lowerToleranceRot,
      double upperToleranceRot,
      boolean requireInTolerance
  ) {
    _roller = roller.getRoller();
    _relativeMovementRot = relativeMovementRot;
    _lowerToleranceRot = lowerToleranceRot;
    _upperToleranceRot = upperToleranceRot;
    _requireInTolerance = requireInTolerance;
  }

  public RollerRotateXCommand(TagalongRoller roller, double relativeMovementRot) {
    this(
        roller,
        relativeMovementRot,
        roller._defaultRollerLowerToleranceRot,
        roller._defaultRollerUpperToleranceRot,
        false
    );
  }

  public RollerRotateXCommand(
      TagalongRoller roller,
      double relativeMovementRot,
      double lowerToleranceRot,
      double upperToleranceRot,
      boolean requireInTolerance
  ) {
    _roller = roller;
    _relativeMovementRot = relativeMovementRot;
    _lowerToleranceRot = lowerToleranceRot;
    _upperToleranceRot = upperToleranceRot;
    _requireInTolerance = requireInTolerance;
  }

  @Override
  public void initialize() {
    _startPosition = _roller.getRollerPosition();
    _goalPosition = _startPosition + _relativeMovementRot;
    _lowerBound = _goalPosition - Math.abs(_lowerToleranceRot);
    _upperBound = _goalPosition + Math.abs(_upperToleranceRot);
    _roller.setRollerProfile(_goalPosition, 0.0);
  }

  @Override
  public void execute() {
    _roller.followLastRollerProfile();
  }

  @Override
  public void end(boolean interrupted) {
    _roller.setRollerPower(0.0);
  }

  @Override
  public boolean isFinished() {
    return _roller.isRollerProfileFinished()
        && (!_requireInTolerance || _roller.inTolerance(_goalPosition, _lowerBound, _upperBound));
  }
}
