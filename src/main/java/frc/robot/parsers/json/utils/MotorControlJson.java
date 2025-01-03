package frc.robot.parsers.json.utils;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class MotorControlJson {
  public boolean clockwisePositive;
  public boolean brakeMode;
  public CurrentLimitsJson motorCurrentLimits;
  public FeedForwardConstantsJson feedforward = new FeedForwardConstantsJson();
  public ClosedLoopConfigsJson closedLoopConfigs = new ClosedLoopConfigsJson();

  public MotorConfigJson motor;

  private TalonFXConfiguration motorConfiguration = null;
  private MotorOutputConfigs motorOutputConfigs = null;
  private Slot0Configs slot0 = null;
  private Slot1Configs slot1 = null;
  private Slot2Configs slot2 = null;

  public TalonFXConfiguration getFullMotorConfiguration() {
    if (motorConfiguration == null) {
      motorConfiguration =
          new TalonFXConfiguration()
              .withSlot0(getSlot0())
              .withSlot1(getSlot1())
              .withSlot2(getSlot2())
              .withCurrentLimits(motorCurrentLimits.getTalonFXCurrentLimits())
              .withMotorOutput(getMotorOutputConfigs())
              .withClosedLoopRamps(closedLoopConfigs.getClosedLoopRampsConfigs())
              .withClosedLoopGeneral(closedLoopConfigs.getClosedLoopGeneralConfigs());
    }
    return motorConfiguration;
  }

  public MotorOutputConfigs getMotorOutputConfigs() {
    if (motorOutputConfigs == null) {
      motorOutputConfigs = new MotorOutputConfigs()
                               .withInverted(getInvertedValue(clockwisePositive))
                               .withNeutralMode(getNeutralMode(brakeMode))
                               .withDutyCycleNeutralDeadband(0.0);
    }
    return motorOutputConfigs;
  }

  public Slot0Configs getSlot0() {
    if (slot0 == null) {
      slot0 = motor.getSlot0Configuration();
    }
    return slot0;
  }

  public Slot1Configs getSlot1() {
    if (slot1 == null) {
      slot1 = motor.getSlot1Configuration();
    }
    return slot1;
  }

  public Slot2Configs getSlot2() {
    if (slot2 == null) {
      slot2 = motor.getSlot2Configuration();
    }
    return slot2;
  }

  public InvertedValue getInvertedValue() {
    return getInvertedValue(clockwisePositive);
  }

  public static InvertedValue getInvertedValue(boolean clockwisePositive) {
    return clockwisePositive ? InvertedValue.Clockwise_Positive
                             : InvertedValue.CounterClockwise_Positive;
  }

  public NeutralModeValue getNeutralMode() {
    return getNeutralMode(brakeMode);
  }

  public static NeutralModeValue getNeutralMode(boolean brakeMode) {
    return brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
  }
}
