package frc.robot.subsystems.minor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.parsers.ElevatorParser;

public class TagalongDualMotorElevator extends TagalongElevator {
  /* -------- Hardware: motors and sensors -------- */
  private final TalonFX _elevatorFollowerMotor;
  private final TalonFXConfiguration _elevatorFollowerMotorConfig;

  public TagalongDualMotorElevator(ElevatorParser parser) {
    super(parser);
    if (_configuredMinorSystemDisable) {
      _elevatorFollowerMotor = null;
      _elevatorFollowerMotorConfig = null;
      return;
    }

    _elevatorFollowerMotor = _elevatorConf.followerMotor.getTalonFX();
    _elevatorFollowerMotorConfig = _elevatorConf.followerMotorControl.getFullMotorConfiguration();
    configFollowerMotor();
  }

  @Override
  public void onEnable() {
    if (_isMinorSystemDisabled) {
      return;
    }
    super.onEnable();
    _elevatorFollowerMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void onDisable() {
    if (_isMinorSystemDisabled) {
      return;
    }
    super.onDisable();
    _elevatorFollowerMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  protected void configFollowerMotor() {
    _elevatorFollowerMotor.getConfigurator().apply(_elevatorFollowerMotorConfig);
    _elevatorFollowerMotor.setControl(new StrictFollower(_elevatorMotor.getDeviceID()));
  }

  @Override
  public boolean motorResetConfig() {
    if (_elevatorFollowerMotor.hasResetOccurred()) {
      configFollowerMotor();
      return true;
    }

    return super.motorResetConfig();
  }
}
