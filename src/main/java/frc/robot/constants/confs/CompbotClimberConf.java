package frc.robot.constants.confs;

import tagalong.subsystems.micro.confs.ElevatorConf;
import tagalong.subsystems.micro.confs.RollerConf;

public class CompbotClimberConf extends BaseClimberConf {
  public static final ElevatorConf elevatorConf = CompbotClimberElevatorConf.construct();
  public static final RollerConf rollerConf = CompbotClimberRollerConf.construct();
  public static final ElevatorConf hooksConf = CompbotClimberHooksConf.construct();
  public static final int breakBeamChannel = -1;

  public static CompbotClimberConf construct() {
    return new CompbotClimberConf(elevatorConf, rollerConf, hooksConf, breakBeamChannel);
  }

  public CompbotClimberConf(
      ElevatorConf elevatorConf, RollerConf rollerConf, ElevatorConf hooksConf, int breakBeamChannel
  ) {
    super(elevatorConf, rollerConf, hooksConf, breakBeamChannel);
  }
}
