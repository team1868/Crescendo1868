package frc.robot.commands.complex;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.enums.DriveModes;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import tagalong.subsystems.micro.Elevator;
import tagalong.subsystems.micro.Pivot;
import tagalong.subsystems.micro.Roller;

public class ResetAllCommand extends Command {
  private final Drivetrain _drivetrain;

  private final Intake _intake;
  private final Pivot _intakePivot;
  private final Roller _intakeRoller;

  private final Shooter _shooter;
  private final Roller _shooterFlywheel;
  private final Pivot _shooterPivot;
  private final Roller _shooterRoller;

  private final Climber _climber;
  private final Elevator _climberElevator;
  private final Elevator _climberHooks;

  public ResetAllCommand(Drivetrain drivetrain, Intake intake, Shooter shooter, Climber climber) {
    _drivetrain = drivetrain;
    _intake = intake;
    _climber = climber;
    _shooter = shooter;

    _intakePivot = intake.getPivot();
    _intakeRoller = intake.getRoller();

    _shooterFlywheel = shooter.getRoller(ShooterConstants.FLYWHEEL_ID);
    _shooterPivot = shooter.getPivot();
    _shooterRoller = shooter.getRoller(ShooterConstants.FEEDER_ID);

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
      _intakePivot.setFollowProfile(true);
      _intakeRoller.setRollerPower(0.0);
      _intakePivot.getPrimaryMotor().setNeutralMode(NeutralModeValue.Brake);
    }

    if (!_shooter._configuredDisable) {
      _shooterFlywheel.setRollerPower(0.0);
      _shooterPivot.setPivotPower(0.0);
      _shooterPivot.setFollowProfile(true);
      _shooterRoller.setRollerPower(0.0);
      _shooterPivot.getPrimaryMotor().setNeutralMode(NeutralModeValue.Brake);
    }

    if (!_climber._configuredDisable) {
      _climberElevator.setPrimaryPower(0.0);
      _climberElevator.setFollowProfile(true);
      _climberElevator.getPrimaryMotor().setNeutralMode(NeutralModeValue.Brake);

      _climberHooks.setPrimaryPower(0.0);
      _climberHooks.setFollowProfile(false);
      _climberHooks.getPrimaryMotor().setNeutralMode(NeutralModeValue.Brake);
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (!_intake._configuredDisable) {
      _intakePivot.setPivotProfile(_intakePivot.getPivotPosition(), 0.0);
      _intakePivot.getPrimaryMotor().setControl(new PositionVoltage(_intakePivot.getPivotPosition())
      );
    }

    if (!_shooter._configuredDisable) {
      _shooterPivot.setPivotProfile(_shooterPivot.getPivotPosition(), 0.0);
      _shooterPivot.getPrimaryMotor().setControl(new PositionVoltage(_shooterPivot.getPivotPosition(
      )));
    }

    if (!_climber._configuredDisable) {
      _climberElevator.setElevatorProfile(_climberElevator.getElevatorHeightM(), 0.0);
      _climberElevator.getPrimaryMotor().setControl(
          new PositionVoltage(_climberElevator.getPrimaryMotorPosition())
      );

      _climberHooks.setElevatorProfile(_climberHooks.getElevatorHeightM(), 0.0);
      _climberHooks.getPrimaryMotor().setControl(
          new PositionVoltage(_climberHooks.getPrimaryMotorPosition())
      );
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
