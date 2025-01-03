package frc.robot.constants.confs;

import tagalong.subsystems.micro.confs.ElevatorConf;
import tagalong.subsystems.micro.confs.RollerConf;

public class BaseClimberConf {
  public ElevatorConf elevatorConf;
  public RollerConf rollerConf;
  public ElevatorConf hooksConf;
  public int breakBeamChannel;

  public BaseClimberConf(
      ElevatorConf elevatorConf, RollerConf rollerConf, ElevatorConf hooksConf, int breakBeamChannel
  ) {
    this.elevatorConf = elevatorConf;
    this.rollerConf = rollerConf;
    this.hooksConf = hooksConf;
    this.breakBeamChannel = breakBeamChannel;
  }
}
