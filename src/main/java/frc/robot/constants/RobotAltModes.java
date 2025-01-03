package frc.robot.constants;

import frc.robot.Robot;
import java.util.Arrays;
import java.util.List;

// TODO rename to TagalongRobotModes
public final class RobotAltModes {
  /* --- Test Mode Configurations --- */
  public static final boolean isTestMode = false;
  public static final double TEST_MODE_COEFFICIENT = 0.5;

  /* --- Debug Modes --- */
  public static final List<String> selectedMinisystems = Arrays.asList("Climber Hooks");
  public static final boolean isPIDTuningMode = false;
  public static final boolean isFFTuningMode = false;
  public static final boolean isVerboseMode = true;
  public static final boolean isReplayMode = false;
  public static final boolean isLoopTiming = false;
  public static final boolean isAutoTuning = false;
  public static final boolean isPoseTuning = false;
  public static final boolean isUnprofiledPIDMode = false;
  public static final boolean isModuleTuningMode = false;
  public static final boolean isSOTFTuningMode = false;

  /* --- ODOMETRY CONFIGURATIONS --- */
  // TODO: move to constants
  public static final boolean kEnableCouplingCompensation = false;
  public static final boolean kEnableLatencyCompensation = true;
  public static final boolean kEnableThreadedOdometry = true;
  // Hz, only applies if threaded odometry is enabled
  public static final double kOdometryFrequency = 250.0;
  public static final boolean kEnableSynchronousOutput = true;
  public static final double steerDriveCouplingRatio =
      RobotAltModes.kEnableCouplingCompensation ? 50.0 / 14.0 : 0.0;
}
