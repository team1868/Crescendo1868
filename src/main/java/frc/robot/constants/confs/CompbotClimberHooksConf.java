package frc.robot.constants.confs;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import tagalong.controls.FeedforwardConstants;
import tagalong.controls.PIDSGVAConstants;
import tagalong.devices.Motors;
import tagalong.subsystems.micro.confs.ElevatorConf;
import tagalong.units.AccelerationUnits;
import tagalong.units.DistanceUnits;
import tagalong.units.MassUnits;
import tagalong.units.VelocityUnits;

public class CompbotClimberHooksConf extends ElevatorConf {
  private static final String name = "Climber Hooks";

  private static final Motors[] motorTypes = {Motors.KRAKEN_X60_FOC}; // primary motor is the
  // first in the list
  private static final int[] motorDeviceIDs = {45};
  private static final String[] motorCanBus = {"Default Name"};
  private static final InvertedValue[] motorDirection = {InvertedValue.CounterClockwise_Positive};
  private static final NeutralModeValue[] motorEnabledBrakeMode = {NeutralModeValue.Brake};
  private static final NeutralModeValue[] motorDisabledBrakeMode = {NeutralModeValue.Brake};

  private static final DistanceUnits drumDiameterUnits = DistanceUnits.INCH;
  private static final double drumDiameter = 0.85;

  private static final int[][] gearRatio = {{48, 14}, {34, 16}};

  /* -------- Required Tuning -------- */

  private static final DistanceUnits trapezoidalLimitsUnits = DistanceUnits.METER;
  private static final VelocityUnits trapezoidalLimitsUnitsVelocity =
      VelocityUnits.METERS_PER_SECOND;
  private static final AccelerationUnits trapezoidalLimitsUnitsAcceleration =
      AccelerationUnits.METERS_PER_SECOND2;
  private static final double trapezoidalLimitsVelocity = 0.3;
  private static final double trapezoidalLimitsAcceleration = 1.0;

  private static final DistanceUnits positionalLimitsUnits = DistanceUnits.METER;
  private static final double positionalMin = 0.0;
  private static final double positionalMax = 0.836826;

  private static final DistanceUnits defaultTolerancesUnits = DistanceUnits.METER;
  private static final double defaultLowerTolerance = 0.05;
  private static final double defaultUpperTolerance = 0.05;

  private static final boolean enableStatorCurrentLimit = true;
  private static final int motorCurrentLimitStatorPeakLimit = 80;
  private static final boolean enableSupplyCurrentLimit = true;
  private static final int motorCurrentLimitSupplyPeakLimit = 80;
  private static final int motorCurrentLimitSupplyContinuousLimit = 60;
  private static final double motorCurrentLimitPeakDuration = 0.5;

  private static final FeedforwardConstants feedForward =
      new FeedforwardConstants(0.2675, 0.0175, 10.0, 0.0);

  /* -------- Positional -------- */
  private static final PIDSGVAConstants slot0 =
      new PIDSGVAConstants(5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  /* -------- Velocity -------- */
  private static final PIDSGVAConstants slot1 =
      new PIDSGVAConstants(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  /* -------- Current -------- */
  private static final PIDSGVAConstants slot2 =
      new PIDSGVAConstants(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  /* -------- Simulation Specific Control -------- */
  private static final MassUnits carriageMassUnits = MassUnits.POUNDS;
  private static final double carriageMassValue = 2.0;
  private static final int mech2dDim = 80;
  private static final int lineLength = 30;
  private static final int angle = 90;
  private static final FeedforwardConstants simFeedForward =
      new FeedforwardConstants(0.0, 0.0, 0.0, 0.0);
  /* -------- Positional -------- */
  private static final PIDSGVAConstants simSlot0 =
      new PIDSGVAConstants(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  /* -------- Velocity -------- */
  private static final PIDSGVAConstants simSlot1 =
      new PIDSGVAConstants(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  /* -------- Current -------- */
  private static final PIDSGVAConstants simSlot2 =
      new PIDSGVAConstants(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  public static CompbotClimberElevatorConf construct() {
    return new CompbotClimberElevatorConf(
        name,
        motorTypes,
        motorDeviceIDs,
        motorCanBus,
        motorDirection,
        motorEnabledBrakeMode,
        motorDisabledBrakeMode,
        gearRatio,
        positionalLimitsUnits,
        positionalMin,
        positionalMax,
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
        carriageMassUnits,
        carriageMassValue,
        mech2dDim,
        lineLength,
        angle,
        simSlot0,
        simSlot1,
        simSlot2,
        drumDiameterUnits,
        drumDiameter
    );
  }

  public CompbotClimberHooksConf(
      String name,
      Motors[] motorTypes,
      int[] motorDeviceIDs,
      String[] motorCanBus,
      InvertedValue[] motorDirection,
      NeutralModeValue[] motorEnabledBrakeMode,
      NeutralModeValue[] motorDisabledBrakeMode,
      int[][] gearRatio,
      DistanceUnits positionalLimitsUnits,
      double positionalMin,
      double positionalMax,
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
      MassUnits carriageMassUnit,
      double carriageMassValue,
      int mech2dDim,
      int lineLength,
      int angle,
      PIDSGVAConstants simSlot0,
      PIDSGVAConstants simSlot1,
      PIDSGVAConstants simSlot2,
      DistanceUnits drumDiameterUnits,
      double drumDiameter
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
        positionalLimitsUnits,
        positionalMin,
        positionalMax,
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
        carriageMassUnit,
        carriageMassValue,
        mech2dDim,
        lineLength,
        angle,
        simSlot0,
        simSlot1,
        simSlot2,
        drumDiameterUnits,
        drumDiameter
    );
  }
}
