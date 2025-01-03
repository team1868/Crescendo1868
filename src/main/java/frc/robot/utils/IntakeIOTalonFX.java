package frc.robot.utils;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import frc.robot.parsers.json.IntakeConfJson;
import frc.robot.subsystems.Intake;
import java.util.function.Supplier;
import tagalong.subsystems.micro.Pivot;
import tagalong.subsystems.micro.Roller;

public class IntakeIOTalonFX implements IntakeIO {
  private final Intake intake;
  private final Pivot intakePivot;
  private final Roller intakeRoller;

  private TalonFX pivotMotor;
  private TalonFX rollerMotor;

  private Supplier<Boolean> isPieceInIntake;

  private StatusSignal<Double> pivotPositionRot;
  private StatusSignal<Double> pivotVelocityRPS;
  private StatusSignal<Double> pivotAppliedVolts;
  private StatusSignal<Double> pivotCurrentAmps;

  private StatusSignal<Double> rollerVelocityRPS;
  private StatusSignal<Double> rollerAppliedVolts;
  private StatusSignal<Double> rollerCurrentAmps;

  public IntakeIOTalonFX(Intake intake) {
    this.intake = intake;
    intakePivot = intake.getPivot();
    intakeRoller = intake.getRoller();

    if (!intake._configuredDisable) {
      pivotMotor = intakePivot.getPrimaryMotor();
      rollerMotor = intakeRoller.getPrimaryMotor();
      isPieceInIntake = intake::isPieceInIntake;

      pivotPositionRot = pivotMotor.getPosition().clone();
      pivotVelocityRPS = pivotMotor.getVelocity().clone();
      pivotAppliedVolts = pivotMotor.getMotorVoltage().clone();
      pivotCurrentAmps = pivotMotor.getStatorCurrent().clone();

      rollerVelocityRPS = rollerMotor.getVelocity().clone();
      rollerAppliedVolts = rollerMotor.getMotorVoltage().clone();
      rollerCurrentAmps = rollerMotor.getStatorCurrent().clone();
    }
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    if (!intake._configuredDisable) {
      // inputs.connected = BaseStatusSignal
      // .refreshAll(new BaseStatusSignal[] {
      // pivotPositionRot,
      // pivotVelocityRPS,
      // pivotAppliedVolts,
      // pivotCurrentAmps,
      // rollerVelocityRPS,
      // rollerAppliedVolts,
      // rollerCurrentAmps})
      // .equals(StatusCode.OK);
      inputs.isPieceInIntake = isPieceInIntake.get();

      inputs.pivotPositionRad = Units.rotationsToRadians(pivotPositionRot.getValueAsDouble());
      inputs.pivotVelocityRadPerSec = Units.rotationsToRadians(pivotVelocityRPS.getValueAsDouble());
      inputs.pivotAppliedVolts = pivotAppliedVolts.getValueAsDouble();
      inputs.pivotCurrentAmps = pivotCurrentAmps.getValueAsDouble();

      inputs.rollerVelocityRadPerSec =
          Units.rotationsToRadians(rollerVelocityRPS.getValueAsDouble());
      inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
      inputs.rollerCurrentAmps = rollerCurrentAmps.getValueAsDouble();
    }
  }
}
