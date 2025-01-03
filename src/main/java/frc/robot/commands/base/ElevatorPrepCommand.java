package frc.robot.commands.base;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ElevatorConstants;
import tagalong.subsystems.TagalongSubsystemBase;
import tagalong.subsystems.micro.Elevator;
import tagalong.subsystems.micro.augments.ElevatorAugment;

public class ElevatorPrepCommand<T extends TagalongSubsystemBase & ElevatorAugment>
    extends Command {
  private final Elevator _elevator;

  public ElevatorPrepCommand(int id, T elevator) {
    _elevator = elevator.getElevator(id);

    addRequirements(elevator);
  }

  public ElevatorPrepCommand(T elevator) {
    this(0, elevator);
  }

  @Override
  public void initialize() {
    _elevator.setPrimaryPower(ElevatorConstants.ELEVATOR_ZEROING_SPEED_MPS);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    _elevator.setPrimaryPower(0.0);
    _elevator.setElevatorHeight(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
