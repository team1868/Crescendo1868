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

public final class CompbotShooterRollerConf extends RollerConf {
  private static final String name = "Shooter Roller";

  private static final Motors[] motorTypes = {Motors.KRAKEN_X60_FOC};
  private static final int[] motorDeviceIDs = {34};
  private static final String[] motorCanBus = {"rio"};
  private static final InvertedValue[] motorDirection = {InvertedValue.CounterClockwise_Positive};
  private static final NeutralModeValue[] motorEnabledBrakeMode = {NeutralModeValue.Brake};
  private static final NeutralModeValue[] motorDisabledBrakeMode = {NeutralModeValue.Brake};

  private static final int[][] gearRatio = {{18, 12}};

  private static final DistanceUnits trapezoidalLimitsUnits = DistanceUnits.ROTATION;
  private static final VelocityUnits trapezoidalVelocityUnits = VelocityUnits.ROTATIONS_PER_SECOND;
  private static final double trapezoidalLimitsVelocity = 100.0;
  private static final AccelerationUnits trapezoidalAccelerationUnits =
      AccelerationUnits.ROTATIONS_PER_SECOND2;
  private static final double trapezoidalLimitsAcceleration = 500.0;

  private static final DistanceUnits defaultTolerancesUnit = DistanceUnits.ROTATION;
  private static final double defaultLowerTolerance = 0.3;
  private static final double defaultUpperTolerance = 0.3;

  private static final boolean enableStatorCurrentLimit = true;
  private static final int motorCurrentLimitStatorPeakLimit = 40;
  private static final boolean enableSupplyCurrentLimit = true;
  private static final int motorCurrentLimitSupplyPeakLimit = 40;
  private static final int motorCurrentLimitSupplyContinuousLimit = 40;
  private static final double motorCurrentLimitPeakDuration = 0.1;

  private static final FeedforwardConstants feedForward =
      new FeedforwardConstants(0.27, 0.0, 0.125, 0.0);

  /* -------- Positional -------- */
  private static final PIDSGVAConstants slot0 =
      new PIDSGVAConstants(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
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

  public static CompbotShooterRollerConf construct() {
    return new CompbotShooterRollerConf(
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

  public CompbotShooterRollerConf(
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
