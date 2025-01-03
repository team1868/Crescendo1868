package frc.robot.commands.complex;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.enums.DriveModes;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.minor.TagalongDualMotorFlywheel;
import frc.robot.subsystems.minor.TagalongElevator;
import frc.robot.subsystems.minor.TagalongPivot;
import frc.robot.subsystems.minor.TagalongRoller;

public class ResetAllCommand extends Command {
  private final Drivetrain _drivetrain;

  private final Intake _intake;
  private final TagalongPivot _intakePivot;
  private final TagalongRoller _intakeRoller;

  private final Shooter _shooter;
  private final TagalongDualMotorFlywheel _shooterFlywheel;
  private final TagalongPivot _shooterPivot;
  private final TagalongRoller _shooterRoller;

  private final Climber _climber;
  private final TagalongElevator _climberElevator;
  private final TagalongElevator _climberHooks;

  public ResetAllCommand(Drivetrain drivetrain, Intake intake, Shooter shooter, Climber climber) {
    _drivetrain = drivetrain;
    _intake = intake;
    _climber = climber;
    _shooter = shooter;

    _intakePivot = intake.getPivot();
    _intakeRoller = intake.getRoller();

    _shooterFlywheel = shooter.getFlywheel();
    _shooterPivot = shooter.getPivot();
    _shooterRoller = shooter.getRoller();

    _climberElevator = climber.getElevator(Climber.ElevatorConstants.ELEVATOR_ID);
    _climberHooks = climber.getElevator(Climber.ElevatorConstants.HOOKS_ID);

    addRequirements(drivetrain, intake, shooter, climber);
  }

  @Override
  public void initialize() {
    if (!_drivetrain._configuredDisable)
      _drivetrain.drive(0.0, 0.0, 0.0, DriveModes.FIELD_RELATIVE_ANTISCRUB);

    if (!_intake._configuredDisable) {
      _intakePivot.setPivotPower(0.0);
      _intakePivot.setHoldPivotPosition(true);
      _intakeRoller.setRollerPower(0.0);
      _intakePivot.getPivotMotor().setNeutralMode(NeutralModeValue.Brake);
    }

    if (!_shooter._configuredDisable) {
      _shooterFlywheel.setFlywheelPower(0.0);
      _shooterPivot.setPivotPower(0.0);
      _shooterPivot.setHoldPivotPosition(true);
      _shooterRoller.setRollerPower(0.0);
      _shooterPivot.getPivotMotor().setNeutralMode(NeutralModeValue.Brake);
    }

    if (!_climber._configuredDisable) {
      _climberElevator.setElevatorPower(0.0);
      _climberElevator.setHoldElevatorPosition(true);
      _climberElevator.getElevatorMotor().setNeutralMode(NeutralModeValue.Brake);

      _climberHooks.setElevatorPower(0.0);
      _climberHooks.setHoldElevatorPosition(false);
      _climberHooks.getElevatorMotor().setNeutralMode(NeutralModeValue.Brake);
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (!_intake._configuredDisable) {
      _intakePivot.getPivotMotor().setControl(new PositionVoltage(_intakePivot.getPivotPosition()));
      _intakePivot.setPivotProfile(_intakePivot.getPivotPosition(), 0.0);
    }

    if (!_shooter._configuredDisable) {
      _shooterPivot.getPivotMotor().setControl(new PositionVoltage(_shooterPivot.getPivotPosition())
      );
      _shooterPivot.setPivotProfile(_shooterPivot.getPivotPosition(), 0.0);
    }

    if (!_climber._configuredDisable) {
      _climberElevator.getElevatorMotor().setControl(
          new PositionVoltage(_climberElevator.getElevatorPosition())
      );
      _climberElevator.setElevatorProfile(_climberElevator.getElevatorHeightM(), 0.0);

      _climberHooks.getElevatorMotor().setControl(
          new PositionVoltage(_climberHooks.getElevatorPosition())
      );
      _climberHooks.setElevatorProfile(_climberHooks.getElevatorHeightM(), 0.0);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
