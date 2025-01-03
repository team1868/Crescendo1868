package frc.robot.constants.enums;
import frc.robot.utils.TagalongHeight;

public enum HookPositions implements TagalongHeight {
  DEPLOY(0.836826),
  STOW(0.0),
  CLIMB(0.125590),
  ZERO(0.0);

  private final double _meters;

  HookPositions(double meters) {
    _meters = meters;
  }
  @Override
  public double getHeightM() {
    return _meters;
  }
}
