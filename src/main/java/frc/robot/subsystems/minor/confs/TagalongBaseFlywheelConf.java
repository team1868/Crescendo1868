package frc.robot.subsystems.minor.confs;

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

public class TagalongBaseFlywheelConf extends RollerConf {
  public TagalongBaseFlywheelConf(
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
        VelocityUnits.ROTATIONS_PER_SECOND,
        trapezoidalVelocityUnits.convertX(
            trapezoidalLimitsVelocity, VelocityUnits.ROTATIONS_PER_SECOND
        ),
        AccelerationUnits.ROTATIONS_PER_SECOND2,
        trapezoidalAccelerationUnits.convertX(
            trapezoidalLimitsAcceleration, AccelerationUnits.ROTATIONS_PER_SECOND2
        ),
        DistanceUnits.ROTATION,
        defaultTolerancesUnit.convertX(defaultLowerTolerance, DistanceUnits.ROTATION),
        defaultTolerancesUnit.convertX(defaultUpperTolerance, DistanceUnits.ROTATION),
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
        rollerMOI
    );

    for (int i = 0; i < numMotors; i++) {
      motorConfig[i].MotorOutput.PeakReverseDutyCycle = 0.0;
      motorConfig[i].Voltage.PeakReverseVoltage = 0.0;
    }
  }
}
