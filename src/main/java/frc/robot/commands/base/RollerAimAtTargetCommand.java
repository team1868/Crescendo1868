package frc.robot.commands.base;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import tagalong.math.AlgebraicUtils;
import tagalong.subsystems.TagalongSubsystemBase;
import tagalong.subsystems.micro.Roller;
import tagalong.subsystems.micro.augments.RollerAugment;

public class RollerAimAtTargetCommand<T extends TagalongSubsystemBase & RollerAugment>
    extends Command {
  private final Roller _roller;
  private double _targetRot;
  private boolean _holdPositionAfter;
  private double _lowerBound;
  private double _upperBound;
  private double _maxVelocity;
  protected Translation2d _target;
  private boolean _startedMovement;
  private double _goalVelocityRPS = 0.0;

  public RollerAimAtTargetCommand(
      int i,
      T roller,
      Pose2d curPos,
      Translation2d target,
      boolean holdPositionAfter,
      double maxVelocityRPS,
      double lowerToleranceRot,
      double upperToleranceRot
  ) {
    _roller = roller.getRoller();
    _target = target;
    _targetRot = Math.atan((target.getX() - curPos.getX()) / (target.getY() - curPos.getY()));
    _targetRot += (_targetRot > 0.0) ? 0.0 : 2.0 * Math.PI;
    _holdPositionAfter = holdPositionAfter;
    _maxVelocity = maxVelocityRPS;
    _lowerBound = AlgebraicUtils.cppMod(_targetRot, 1.0) - Math.abs(lowerToleranceRot);
    _upperBound = AlgebraicUtils.cppMod(_targetRot, 1.0) + Math.abs(upperToleranceRot);
    addRequirements(roller);
  }

  public RollerAimAtTargetCommand(
      T roller,
      Pose2d curPos,
      Translation2d target,
      boolean holdPositionAfter,
      double maxVelocityRPS,
      double lowerToleranceRot,
      double upperToleranceRot
  ) {
    this(
        0,
        roller,
        curPos,
        target,
        holdPositionAfter,
        maxVelocityRPS,
        lowerToleranceRot,
        upperToleranceRot
    );
  }

  @Override
  public void initialize() {
    _roller.setHoldPosition(false);
    _startedMovement = false;
  }

  @Override
  public void execute() {
    if (!_startedMovement) {
      _startedMovement = true;
      _roller.setRollerProfile(_targetRot, _goalVelocityRPS);
    }

    if (_startedMovement) {
      _roller.followLastProfile();
    }
  }

  @Override
  public void end(boolean interrupted) {
    _roller.setHoldPosition(_holdPositionAfter);
  }

  @Override
  public boolean isFinished() {
    return _roller.isProfileFinished() && _roller.isRollerInTolerance(_lowerBound, _upperBound);
  }

  public double getRotations(double value) {
    return Units.radiansToRotations(value);
  }
}
