package frc.robot.utils;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Climber;
import tagalong.subsystems.micro.Elevator;

public class ClimberIOTalonFX implements ClimberIO {
  private final Climber climber;
  private final Elevator climberElevator;
  private final TalonFX motor;

  private final StatusSignal<Double> positionRot;
  private final StatusSignal<Double> velocityRPS;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> currentAmps;

  public ClimberIOTalonFX(Climber climber) {
    this.climber = climber;
    climberElevator = climber.getElevator();
    motor = climberElevator.getPrimaryMotor();

    positionRot = motor.getPosition().clone();
    velocityRPS = motor.getVelocity().clone();
    appliedVolts = motor.getMotorVoltage().clone();
    currentAmps = motor.getStatorCurrent().clone();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    if (!climber._configuredDisable) {
      // inputs.connected = BaseStatusSignal
      // .refreshAll(new BaseStatusSignal[] {
      // positionRot, velocityRPS, appliedVolts, currentAmps})
      // .equals(StatusCode.OK);

      inputs.positionRad = Units.rotationsToRadians(positionRot.getValueAsDouble());
      inputs.velocityRadPerSec = Units.rotationsToRadians(velocityRPS.getValueAsDouble());
      inputs.appliedVolts = appliedVolts.getValueAsDouble();
      inputs.currentAmps = currentAmps.getValueAsDouble();
    }
  }
}
