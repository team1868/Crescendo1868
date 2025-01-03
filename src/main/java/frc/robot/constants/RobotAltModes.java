package frc.robot.constants;

import frc.robot.Robot;

public final class RobotAltModes {
  /* --- Test Mode Configurations --- */
  public static final boolean isTestMode = false;
  public static final double TEST_MODE_COEFFICIENT = 0.5;

  /* --- Debug Modes --- */
  public static final String currentMicrosystem = "Climber Hooks";
  public static final boolean isVerboseMode = true;
  public static final boolean isReplayMode = false;
  public static final boolean isLoopTiming = false;
  public static final boolean isAutoTuning = false;
  public static final boolean isPoseTuning = false;
  public static final boolean isPIDTuningMode = false;
  public static final boolean isElevatorTuningMode = false;
  public static final boolean isRollerTuningMode = false;
  public static final boolean isPivotTuningMode = false;
  public static final boolean isFlywheelTuningMode = false;
  public static final boolean isUnprofiledPIDMode = false;
  public static final boolean isModuleTuningMode = false;
  public static final boolean isSOTFTuningMode = false;

  /* --- Sim Actionability --- */
  public static final boolean isSim = Robot.isSimulation();
  public static final boolean isSimElevator = false;

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
