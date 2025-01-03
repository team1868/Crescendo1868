package frc.robot.parsers.json;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.parsers.json.utils.*;

public class SwerveModuleControlJson {
  public String moduleType;
  public MotorControlJson driveControl;
  public MotorControlJson steerControl;

  public TalonFXConfiguration getDriveConfiguration() {
    return driveControl.getFullMotorConfiguration();
  }

  public TalonFXConfiguration getSteerConfiguration() {
    return steerControl.getFullMotorConfiguration();
  }
}
