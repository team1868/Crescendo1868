package frc.robot.constants.enums;

public enum PipelineIndex {
  APRILTAG_3D(0),
  APRILTAG_2D(1),
  COLOR(2);

  public final int index;
  PipelineIndex(int index) {
    this.index = index;
  }
}
