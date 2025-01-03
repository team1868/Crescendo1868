package frc.robot.commands.base;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.enums.LedModes;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Shooter;

public class PreMatchLedCommand extends Command {
  /* Subsystem variables */
  private final Leds _leds;
  private Climber _climber;
  private Shooter _shooter;
  private Intake _intake;
  private Drivetrain _drivetrain;

  /* Other Class variables */
  private Pose2d _goalPose2d; // TODO, get this from currently selected autonomous
  private boolean[] initStatus = new boolean[PreMatchConstants.NUM_SUBSYSTEMS];
  private Transform2d alignment;
  private int[] alignmentStatus = new int[PreMatchConstants.NUM_AXES];

  public PreMatchLedCommand(
      Leds leds,
      Climber climber,
      Shooter shooter,
      Intake intake,
      Drivetrain drivetrain,
      Pose2d goalPose2d
  ) {
    _leds = leds;
    _climber = climber;
    _shooter = shooter;
    _intake = intake;
    _goalPose2d = goalPose2d;
    addRequirements(leds);
    runsWhenDisabled();
  }

  @Override
  public void initialize() {
    for (int i = 0; i < PreMatchConstants.NUM_SECTIONS; i++) {
      _leds.unsafeSetLEDMode(LedModes.RED_SOLID, i);
      if (i < PreMatchConstants.NUM_SUBSYSTEMS) {
        initStatus[i] = false;
      }
      if (i < PreMatchConstants.NUM_AXES) {
        alignmentStatus[i] = -1;
      }
    }
  }

  private void updateStatus() {
    initStatus[0] = _drivetrain.isSubsystemDisabled();
    initStatus[1] = _climber.isSubsystemDisabled();
    initStatus[2] = _shooter.isSubsystemDisabled();
    initStatus[3] = _intake.isSubsystemDisabled();

    alignment = _drivetrain.getPose().minus(_goalPose2d);
    alignmentStatus[0] = findThreshold(Math.abs(alignment.getRotation().getDegrees()), 0);
    alignmentStatus[1] = findThreshold(Math.abs(alignment.getX()), 1);
    alignmentStatus[2] = findThreshold(Math.abs(alignment.getY()), 2);
  }

  private int findThreshold(double value, int axis) {
    for (int i = 0; i < PreMatchConstants.toleranceThresholds.length; i++) {
      if (value <= PreMatchConstants.toleranceThresholds[i][axis]) {
        return i;
      }
    }
    return PreMatchConstants.toleranceThresholds.length - 1;
  }

  @Override
  public void execute() {
    updateStatus();
    // initStatus: drivetrain, climber, shooter, and intake
    // alignmentStatus: r, x, y axis
    // sections: candle, front, left, right, back
    _leds.unsafeSetLEDMode(getCandleLEDMode(), 0); // for candle, just set to getCandle
    if (initStatus[1]) { // for front, check if climber is initialized and blink based on r axis
                         // alignment
      _leds.unsafeSetLEDMode(PreMatchConstants.ledThresholdAnimation[alignmentStatus[0]], 1);
    }
    if (initStatus[2]) { // for right and left, check shooter initialized and blink based on
                         // x axis alignment
      _leds.unsafeSetLEDMode(PreMatchConstants.ledThresholdAnimation[alignmentStatus[1]], 2);
      _leds.unsafeSetLEDMode(PreMatchConstants.ledThresholdAnimation[alignmentStatus[1]], 3);
    }
    if (initStatus[3]) { // for back, check intake initialized and blink based on z axis
                         // alignment
      _leds.unsafeSetLEDMode(PreMatchConstants.ledThresholdAnimation[alignmentStatus[1]], 4);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}

  public int getStartingLocation(double goalPoseY) {
    for (int i = 0; i < PreMatchConstants.STARTING_LOCATIONS_Y.length; i++) {
      if (goalPoseY < PreMatchConstants.STARTING_LOCATIONS_Y[i]) {
        return i;
      }
    }
    return 0;
  }

  public LedModes getCandleLEDMode() {
    return PreMatchConstants.AUTON_STARTING_LOC_MODES[getStartingLocation(_goalPose2d.getY())];
  }

  public static final class PreMatchConstants {
    public static double[] STARTING_LOCATIONS_Y = {0.5, 1, 1.5, 2, 2.5};
    public static LedModes[] AUTON_STARTING_LOC_MODES = {
        LedModes.ORANGE_COLOR_FLOW,
        LedModes.YELLOW_COLOR_FLOW,
        LedModes.GREEN_COLOR_FLOW,
        LedModes.BLUE_COLOR_FLOW,
        LedModes.PURPLE_COLOR_FLOW};

    public static final LedModes[] ledThresholdAnimation = {
        LedModes.SPOOKIES_BLUE_SOLID,
        LedModes.AUTON_BLINKING_1,
        LedModes.AUTON_BLINKING_2,
        LedModes.AUTON_BLINKING_3};

    public static final double[][] toleranceThresholds = {
        {3.0, Units.inchesToMeters(2), Units.inchesToMeters(2)},
        {6.0, Units.inchesToMeters(4), Units.inchesToMeters(4)},
        {9.0, Units.inchesToMeters(6), Units.inchesToMeters(6)},
        {12.0, Units.inchesToMeters(8), Units.inchesToMeters(8)}};
    // 4x3 matrix with each row i having x-axis, y-axis, z-axis tolerances for section i

    /* Constants */
    public static final int NUM_SUBSYSTEMS =
        4; // drivetrain, climber, shooter, intake, in that order
    public static final int NUM_SECTIONS = 5; // candle, front, left, right, back, in that order
    public static final int NUM_AXES = 3; // r-axis, x-axis, y-axis
  }
}
