package frc.robot.constants.enums;

public enum VisionModes {
  PREMATCH(new int[] {3, 4}, new int[] {7, 8}),
  SHOOTING(new int[] {3, 4}, new int[] {7, 8}),
  GROUND_INTAKE(
      PipelineIndex.APRILTAG_3D,
      PipelineIndex.APRILTAG_2D,
      PipelineIndex.COLOR,
      new int[] {},
      new int[] {}
  ),
  CLIMB(
      PipelineIndex.APRILTAG_3D,
      PipelineIndex.APRILTAG_2D,
      PipelineIndex.APRILTAG_2D,
      new int[] {11, 12, 13},
      new int[] {14, 15, 16}
  ),
  AMP(PipelineIndex.APRILTAG_2D,
      PipelineIndex.APRILTAG_3D,
      PipelineIndex.APRILTAG_2D,
      new int[] {5},
      new int[] {6}),
  HP_INTAKE(
      PipelineIndex.APRILTAG_2D, PipelineIndex.APRILTAG_3D, new int[] {9, 10}, new int[] {1, 2}
  ),
  TELEOP(new int[] {}, new int[] {}),
  AUTO(new int[] {}, new int[] {}),
  OFF(new int[] {}, new int[] {});

  public final PipelineIndex primaryPipeline;
  public final PipelineIndex secondaryPipeline;
  public final PipelineIndex colorPipeline;
  public final int[] redTagId;
  public final int[] blueTagId;

  private VisionModes(
      PipelineIndex pipelineP,
      PipelineIndex pipelineS,
      PipelineIndex pipelineC,
      int[] redTagId,
      int[] blueTagId
  ) {
    primaryPipeline = pipelineP;
    secondaryPipeline = pipelineS;
    colorPipeline = pipelineC;
    this.redTagId = redTagId;
    this.blueTagId = blueTagId;
  }
  private VisionModes(
      PipelineIndex pipelineP, PipelineIndex pipelineS, int[] redTagId, int[] blueTagId
  ) {
    this(pipelineP, pipelineS, PipelineIndex.COLOR, redTagId, blueTagId);
  }
  private VisionModes(int[] redTagId, int[] blueTagId) {
    this(PipelineIndex.APRILTAG_3D, PipelineIndex.APRILTAG_2D, redTagId, blueTagId);
  }

  public boolean isGroundIntake() {
    return this == GROUND_INTAKE;
  }

  public static final double PIX_2D_OFFSET = 0.0;
}
