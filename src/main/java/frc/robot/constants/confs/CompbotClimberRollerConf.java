package frc.robot.constants.confs;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import tagalong.controls.FeedforwardConstants;
import tagalong.controls.PIDSGVAConstants;
import tagalong.devices.Motors;
import tagalong.subsystems.micro.confs.RollerConf;
import tagalong.units.AccelerationUnits;
import tagalong.units.DistanceUnits;
import tagalong.units.VelocityUnits;

public class CompbotClimberRollerConf extends RollerConf {
  private static final String name = "Climber Roller";

  private static final Motors[] motorTypes = {Motors.KRAKEN_X60_FOC}; // primary motor is the
  // first in the list
  private static final int[] motorDeviceIDs = {60};
  private static final String[] motorCanBus = {"rio"};
  private static final InvertedValue[] motorDirection = {InvertedValue.CounterClockwise_Positive};
  private static final NeutralModeValue[] motorEnabledBrakeMode = {NeutralModeValue.Coast};
  private static final NeutralModeValue[] motorDisabledBrakeMode = {NeutralModeValue.Coast};

  private static final int[][] gearRatio = {{18, 12}};

  /* -------- Required Tuning -------- */

  private static final DistanceUnits trapezoidalLimitsUnits = DistanceUnits.ROTATION;
  private static final VelocityUnits trapezoidalLimitsUnitsVelocity =
      VelocityUnits.ROTATIONS_PER_SECOND;
  private static final AccelerationUnits trapezoidalLimitsUnitsAcceleration =
      AccelerationUnits.ROTATIONS_PER_SECOND2;
  private static final double trapezoidalLimitsVelocity = 45.0;
  private static final double trapezoidalLimitsAcceleration = 120.0;

  private static final DistanceUnits defaultTolerancesUnits = DistanceUnits.ROTATION;
  private static final double defaultLowerTolerance = 3.0;
  private static final double defaultUpperTolerance = 3.0;

  private static final boolean enableStatorCurrentLimit = true;
  private static final int motorCurrentLimitStatorPeakLimit = 80;
  private static final boolean enableSupplyCurrentLimit = true;
  private static final int motorCurrentLimitSupplyPeakLimit = 80;
  private static final int motorCurrentLimitSupplyContinuousLimit = 80;
  private static final double motorCurrentLimitPeakDuration = 1.0;

  private static final FeedforwardConstants feedForward =
      new FeedforwardConstants(0.273, 0.0, 0.125, 0.0);

  /* -------- Positional -------- */
  private static final PIDSGVAConstants slot0 =
      new PIDSGVAConstants(120.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  /* -------- Velocity -------- */
  private static final PIDSGVAConstants slot1 =
      new PIDSGVAConstants(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  /* -------- Current -------- */
  private static final PIDSGVAConstants slot2 =
      new PIDSGVAConstants(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  /* -------- Simulation Specific Control -------- */
  private static final int simNumLigaments = 3;
  private static final double simMOI = 0.01;
  private static final FeedforwardConstants simFeedForward =
      new FeedforwardConstants(0.0, 0.0, 0.0, 0.0);
  /* -------- Positional -------- */
  private static final PIDSGVAConstants simSlot0 =
      new PIDSGVAConstants(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  /* -------- Velocity -------- */
  private static final PIDSGVAConstants simSlot1 =
      new PIDSGVAConstants(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  /* -------- Current -------- */
  private static final PIDSGVAConstants simSlot2 =
      new PIDSGVAConstants(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  public static CompbotClimberRollerConf construct() {
    return new CompbotClimberRollerConf(
        name,
        motorTypes,
        motorDeviceIDs,
        motorCanBus,
        motorDirection,
        motorEnabledBrakeMode,
        motorDisabledBrakeMode,
        gearRatio,
        trapezoidalLimitsUnits,
        trapezoidalLimitsUnitsVelocity,
        trapezoidalLimitsVelocity,
        trapezoidalLimitsUnitsAcceleration,
        trapezoidalLimitsAcceleration,
        defaultTolerancesUnits,
        defaultLowerTolerance,
        defaultUpperTolerance,
        feedForward,
        simFeedForward,
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(motorCurrentLimitStatorPeakLimit)
            .withSupplyCurrentLimit(motorCurrentLimitSupplyPeakLimit)
            .withSupplyCurrentThreshold(motorCurrentLimitSupplyContinuousLimit)
            .withSupplyTimeThreshold(motorCurrentLimitPeakDuration)
            .withStatorCurrentLimitEnable(enableStatorCurrentLimit)
            .withSupplyCurrentLimitEnable(enableSupplyCurrentLimit),
        slot0,
        slot1,
        slot2,
        simSlot0,
        simSlot1,
        simSlot2,
        simNumLigaments,
        simMOI
    );
  }

  public CompbotClimberRollerConf(
      String name,
      Motors[] motorTypes,
      int[] motorDeviceIDs,
      String[] motorCanBus,
      InvertedValue[] motorDirection,
      NeutralModeValue[] motorEnabledBrakeMode,
      NeutralModeValue[] motorDisabledBrakeMode,
      int[][] gearRatio,
      DistanceUnits trapezoidalLimitsUnits,
      VelocityUnits trapezoidalVelocityUnits,
      double trapezoidalLimitsVelocity,
      AccelerationUnits trapezoidalAccelerationUnits,
      double trapezoidalLimitsAcceleration,
      DistanceUnits defaultTolerancesUnit,
      double defaultLowerTolerance,
      double defaultUpperTolerance,
      FeedforwardConstants feedForward,
      FeedforwardConstants simFeedForward,
      CurrentLimitsConfigs currentLimitsConfigs,
      PIDSGVAConstants slot0,
      PIDSGVAConstants slot1,
      PIDSGVAConstants slot2,
      PIDSGVAConstants simSlot0,
      PIDSGVAConstants simSlot1,
      PIDSGVAConstants simSlot2,
      int simNumLigaments,
      double rollerMOI
  ) {
    super(
        name,
        motorTypes,
        motorDeviceIDs,
        motorCanBus,
        motorDirection,
        motorEnabledBrakeMode,
        motorDisabledBrakeMode,
        gearRatio,
        trapezoidalLimitsUnits,
        trapezoidalVelocityUnits,
        trapezoidalLimitsVelocity,
        trapezoidalAccelerationUnits,
        trapezoidalLimitsAcceleration,
        defaultTolerancesUnit,
        defaultLowerTolerance,
        defaultUpperTolerance,
        feedForward,
        simFeedForward,
        currentLimitsConfigs,
        slot0,
        slot1,
        slot2,
        simSlot0,
        simSlot1,
        simSlot2,
        simNumLigaments,
        simMOI
    );
  }
}
