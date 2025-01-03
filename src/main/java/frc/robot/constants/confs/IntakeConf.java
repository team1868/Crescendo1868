
package frc.robot.constants.confs;

import frc.robot.subsystems.minor.confs.TagalongBaseFlywheelConf;
import tagalong.subsystems.micro.confs.PivotConf;
import tagalong.subsystems.micro.confs.RollerConf;

public class IntakeConf {
  TagalongBaseFlywheelConf flywheelConf;
  PivotConf pivotConf;
  RollerConf rollerConf;
  double pivotUnsafeMinRot;
  double pivotUnsafeMaxRot;

  public IntakeConf(
      PivotConf pivotConf, RollerConf rollerConf, double pivotUnsafeMinRot, double pivotUnsafeMaxRot
  ) {
    this.pivotConf = pivotConf;
    this.rollerConf = rollerConf;
    this.pivotUnsafeMinRot = pivotUnsafeMinRot;
    this.pivotUnsafeMaxRot = pivotUnsafeMaxRot;
  }
}
