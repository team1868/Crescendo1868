package frc.robot.constants.enums;

public enum ShuffleboardStatus {
  SHOW(true),
  HIDE(false);

  public boolean show;
  ShuffleboardStatus(boolean show) {
    this.show = show;
  }
}
