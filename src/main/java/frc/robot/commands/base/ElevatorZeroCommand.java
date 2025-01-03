package frc.robot.commands.base;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ElevatorConstants;
import frc.robot.subsystems.minor.TagalongElevator;
import frc.robot.utils.ElevatorAugment;
import frc.robot.utils.TagalongSubsystemBase;

public class ElevatorZeroCommand<T extends TagalongSubsystemBase & ElevatorAugment>
    extends Command {
  private final TagalongElevator _elevator;
  private double _prevHeightM;
  private double _count;

  public ElevatorZeroCommand(T elevator) {
    _elevator = elevator.getElevator();

    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    _elevator.setElevatorPower(ElevatorConstants.ELEVATOR_ZEROING_SPEED_MPS);
    _prevHeightM = _elevator.getElevatorHeightM();
    _count = 0;
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    _elevator.setElevatorPower(0.0);
    if (!interrupted)
      _elevator.setElevatorHeight(0);
  }

  @Override
  public boolean isFinished() {
    double currentHeightM = _elevator.getElevatorHeightM();

    if (_prevHeightM - currentHeightM > ElevatorConstants.ELEVATOR_ZEROING_STALL_TOLERANCE) {
      _prevHeightM = currentHeightM;
      _count = 0;
      return false;
    } else if (_count++ < ElevatorConstants.ELEVATOR_ZEROING_STALL_LOOPS) {
      _prevHeightM = currentHeightM;
      return false;
    } else {
      return true;
    }
  }
}
