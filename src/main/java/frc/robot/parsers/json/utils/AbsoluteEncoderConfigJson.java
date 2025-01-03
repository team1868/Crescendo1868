package frc.robot.parsers.json.utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class AbsoluteEncoderConfigJson {
  public boolean zeroToOne;
  public UnitJson magnetOffset;
  public boolean clockwisePositive;
  public CANcoderConfiguration cancoderConfiguration;
  public MagnetSensorConfigs magnetSensorConfigs;

  public AbsoluteSensorRangeValue getAbsoluteSensorRange() {
    return zeroToOne ? AbsoluteSensorRangeValue.Unsigned_0To1
                     : AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
  }

  public SensorDirectionValue getSensorDirection() {
    return clockwisePositive ? SensorDirectionValue.Clockwise_Positive
                             : SensorDirectionValue.CounterClockwise_Positive;
  }

  public CANcoderConfiguration getCancoderConfig() {
    if (cancoderConfiguration == null) {
      MagnetSensorConfigs magnetSensorConfigs =
          new MagnetSensorConfigs()
              .withAbsoluteSensorRange(getAbsoluteSensorRange())
              .withMagnetOffset(magnetOffset.getDistRotation().getRotations())
              .withSensorDirection(getSensorDirection());
      cancoderConfiguration = new CANcoderConfiguration().withMagnetSensor(magnetSensorConfigs);
    }
    return cancoderConfiguration;
  }
}
