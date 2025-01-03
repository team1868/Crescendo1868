package frc.robot.constants.confs;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import tagalong.controls.FeedforwardConstants;
import tagalong.controls.PIDSGVAConstants;
import tagalong.devices.Encoders;
import tagalong.devices.Motors;
import tagalong.subsystems.micro.confs.PivotConf;
import tagalong.units.AccelerationUnits;
import tagalong.units.DistanceUnits;
import tagalong.units.VelocityUnits;

// TODO
public class CompbotIntakePivotConf extends PivotConf {
  public static final String name = "IntakePivot";

  public static final Motors[] motorTypes = {Motors.KRAKEN_X60_FOC};
  public static final int[] motorDeviceIDs = {40};
  public static final String[] motorCanBus = {"Default Name"};

  public static final Encoders encoderTypes = Encoders.CANCODER;
  public static final int encoderDeviceID = 42;
  public static final String encoderCanBus = "Default Name";

  public static final InvertedValue[] motorDirection = {InvertedValue.CounterClockwise_Positive};
  public static final NeutralModeValue[] motorEnabledBrakeMode = {NeutralModeValue.Brake};
  public static final NeutralModeValue[] motorDisabledBrakeMode = {NeutralModeValue.Brake};

  public static final int[][] motorToPivotRatio = {{66, 14}, {64, 20}, {60, 16}};
  public static final int[][] encoderToPivotRatio = {{1, 1}};

  public static final DistanceUnits trapezoidalLimitsUnits = DistanceUnits.DEGREE;
  public static final VelocityUnits trapezoidalVelocityUnits = VelocityUnits.DEGREES_PER_SECOND;
  public static final double trapezoidalLimitsVelocity = 1080.0;
  public static final AccelerationUnits trapezoidalAccelerationUnits =
      AccelerationUnits.DEGREES_PER_SECOND2;
  public static final double trapezoidalLimitsAcceleration = 1980.0;

  public static final DistanceUnits positionalLimitsUnits = DistanceUnits.DEGREE;
  public static final double positionalLimitsMin = -8.0;
  public static final double positionalLimitsMax = 71.0;

  public static final DistanceUnits defaultTolerancesUnit = DistanceUnits.DEGREE;
  public static final double defaultLowerTolerance = 1.0;
  public static final double defaultUpperTolerance = 1.0;

  public static final boolean encoderConfigZeroToOne = true;
  public static final boolean encoderConfigClockwisePositive = true;
  public static final DistanceUnits encoderConfigMagnetOffsetUnit = DistanceUnits.ROTATION;
  public static final double encoderConfigMagnetOffsetValue = -0.466796875;

  public static final boolean enableStatorCurrentLimit = true;
  public static final int motorCurrentLimitStatorPeakLimit = 80;
  public static final boolean enableSupplyCurrentLimit = true;
  public static final int motorCurrentLimitSupplyPeakLimit = 80;
  public static final int motorCurrentLimitSupplyContinuousLimit = 40;
  public static final double motorCurrentLimitPeakDuration = 0.1;

  public static final FeedforwardConstants feedForward =
      new FeedforwardConstants(0.0475, 0.3375, 1.15, 0.0);

  /* -------- Positional -------- */
  public static final PIDSGVAConstants slot0 =
      new PIDSGVAConstants(15.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  /* -------- Velocity -------- */
  public static final PIDSGVAConstants slot1 =
      new PIDSGVAConstants(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  /* -------- Current -------- */
  public static final PIDSGVAConstants slot2 =
      new PIDSGVAConstants(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  public static final boolean closedLoopConfigsContinuousWrap = true;

  public static final DistanceUnits ffOffsetUnit = DistanceUnits.DEGREE;
  public static final double ffOffsetValue = 6.0;

  public static final DistanceUnits profileOffsetUnit = DistanceUnits.DEGREE;
  public static final double profileOffsetValue = 0.0;

  /* -------- Simulation Specific Control -------- */
  public static final FeedforwardConstants simFeedForward =
      new FeedforwardConstants(0.0, 0.0, 0.0, 0.0);
  /* -------- Positional -------- */
  public static final PIDSGVAConstants simSlot0 =
      new PIDSGVAConstants(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  /* -------- Velocity -------- */
  public static final PIDSGVAConstants simSlot1 =
      new PIDSGVAConstants(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  /* -------- Current -------- */
  public static final PIDSGVAConstants simSlot2 =
      new PIDSGVAConstants(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  public static final double pivotMOI = 1.0;
  public static final double pivotLengthM = 1.0;

  public static CompbotIntakePivotConf construct() {
    // placeholder
    return new CompbotIntakePivotConf(
        name,
        motorTypes,
        motorDeviceIDs,
        motorCanBus,
        motorDirection,
        encoderTypes,
        encoderDeviceID,
        encoderCanBus,
        encoderConfigZeroToOne,
        encoderConfigClockwisePositive,
        encoderConfigMagnetOffsetUnit,
        encoderConfigMagnetOffsetValue,
        motorEnabledBrakeMode,
        motorDisabledBrakeMode,
        motorToPivotRatio,
        encoderToPivotRatio,
        positionalLimitsUnits,
        positionalLimitsMin,
        positionalLimitsMax,
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
        closedLoopConfigsContinuousWrap,
        ffOffsetUnit,
        ffOffsetValue,
        profileOffsetUnit,
        profileOffsetValue,
        pivotMOI,
        pivotLengthM
    );
  }

  public CompbotIntakePivotConf(
      String name,
      Motors[] motorTypes,
      int[] motorDeviceIDs,
      String[] motorCanBus,
      InvertedValue[] motorDirection,
      Encoders encoderType,
      int encoderDeviceID,
      String encoderCanBus,
      boolean encoderConfigZeroToOne,
      boolean encoderConfigClockwisePositive,
      DistanceUnits encoderConfigMagnetOffsetUnit,
      double encoderConfigMagnetOffsetValue,
      NeutralModeValue[] motorEnabledBrakeMode,
      NeutralModeValue[] motorDisabledBrakeMode,
      int[][] motorToPivotRatio,
      int[][] encoderToPivotRatio,
      DistanceUnits rotationalLimitsUnits,
      double rotationalMin,
      double rotationalMax,
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
      boolean closedLoopConfigsContinuousWrap,
      DistanceUnits ffOffsetUnit,
      double ffOffsetValue,
      DistanceUnits profileOffsetUnit,
      double profileOffsetValue,
      double pivotMOI,
      double pivotLengthM
  ) {
    super(
        name,
        motorTypes,
        motorDeviceIDs,
        motorCanBus,
        motorDirection,
        encoderTypes,
        encoderDeviceID,
        encoderCanBus,
        encoderConfigZeroToOne,
        encoderConfigClockwisePositive,
        encoderConfigMagnetOffsetUnit,
        encoderConfigMagnetOffsetValue,
        motorEnabledBrakeMode,
        motorDisabledBrakeMode,
        motorToPivotRatio,
        encoderToPivotRatio,
        positionalLimitsUnits,
        positionalLimitsMin,
        positionalLimitsMax,
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
        closedLoopConfigsContinuousWrap,
        ffOffsetUnit,
        ffOffsetValue,
        profileOffsetUnit,
        profileOffsetValue,
        pivotMOI,
        pivotLengthM
    );
  }
}
