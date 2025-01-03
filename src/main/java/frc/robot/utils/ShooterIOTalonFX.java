package frc.robot.utils;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.confs.BaseShooterConf;
import frc.robot.parsers.json.ShooterConfJson;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import java.util.function.Supplier;

public class ShooterIOTalonFX implements ShooterIO {
  private final BaseShooterConf conf;
  private final TalonFX pivotMotor;
  private final TalonFX flywheelMotor;

  private final Supplier<Boolean> isPieceInIntake;

  private final StatusSignal<Double> pivotPosition;
  private final StatusSignal<Double> pivotVelocity;
  private final StatusSignal<Double> pivotAppliedVolts;
  private final StatusSignal<Double> pivotCurrent;

  private final StatusSignal<Double> flywheelVelocity;
  private final StatusSignal<Double> flywheelAppliedVolts;
  private final StatusSignal<Double> flywheelCurrent;

  public ShooterIOTalonFX(Shooter shooter) {
    conf = shooter._shooterConf;
    pivotMotor = shooter.getPivot().getPrimaryMotor();
    flywheelMotor = shooter.getRoller(ShooterConstants.FLYWHEEL_ID).getPrimaryMotor();

    isPieceInIntake = shooter::isPieceInChute;

    pivotPosition = pivotMotor.getPosition().clone();
    pivotVelocity = pivotMotor.getVelocity().clone();
    pivotAppliedVolts = pivotMotor.getMotorVoltage().clone();
    pivotCurrent = pivotMotor.getStatorCurrent().clone();

    flywheelVelocity = flywheelMotor.getVelocity().clone();
    flywheelAppliedVolts = flywheelMotor.getMotorVoltage().clone();
    flywheelCurrent = flywheelMotor.getStatorCurrent().clone();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    if (conf != null) {
      // inputs.connected = BaseStatusSignal
      // .refreshAll(new BaseStatusSignal[] {
      // pivotPosition,
      // pivotVelocity,
      // pivotAppliedVolts,
      // pivotCurrent,
      // flywheelVelocity,
      // flywheelAppliedVolts,
      // flywheelCurrent
      // })
      // .equals(StatusCode.OK);

      inputs.isPieceInIntake = isPieceInIntake.get();

      inputs.pivotPositionRad = Units.rotationsToRadians(pivotPosition.getValueAsDouble());
      inputs.pivotVelocityRadPerSec = Units.rotationsToRadians(pivotVelocity.getValueAsDouble());
      inputs.pivotAppliedVolts = pivotAppliedVolts.getValueAsDouble();
      inputs.pivotCurrentAmps = pivotCurrent.getValueAsDouble();

      inputs.flywheelVelocityRadPerSec =
          Units.rotationsToRadians(flywheelVelocity.getValueAsDouble());
      inputs.flywheelAppliedVolts = flywheelAppliedVolts.getValueAsDouble();
      inputs.flywheelCurrentAmps = flywheelCurrent.getValueAsDouble();
    }
  }
}
