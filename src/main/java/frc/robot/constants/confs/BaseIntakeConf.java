package frc.robot.constants.confs;

import tagalong.subsystems.micro.confs.PivotConf;
import tagalong.subsystems.micro.confs.RollerConf;
import tagalong.units.DistanceUnits;

public class BaseIntakeConf {
  public DistanceUnits pivotUnsafePositionalLimitsUnits = DistanceUnits.ROTATION;

  public PivotConf pivotConf = CompbotIntakePivotConf.construct();
  public RollerConf rollerConf = CompbotIntakeRollerConf.construct();
  public String pivotConflict;

  public double pivotUnsafePositionalMin;
  public double pivotUnsafePositionalMax;

  public int breakBeamChannel = 1;

  public BaseIntakeConf(
      PivotConf pivotConf,
      RollerConf rollerConf,
      String pivotConflict,
      DistanceUnits pivotUnsafePositionalLimitsUnits,
      double pivotUnsafePositionalMin,
      double pivotUnsafePositionalMax,
      int breakBeamChannel
  ) {
    this.pivotConf = pivotConf;
    this.rollerConf = rollerConf;
    this.pivotConflict = pivotConflict;
    this.pivotUnsafePositionalLimitsUnits = pivotUnsafePositionalLimitsUnits;
    this.pivotUnsafePositionalMax = pivotUnsafePositionalLimitsUnits.convertX(
        pivotUnsafePositionalMax, this.pivotUnsafePositionalLimitsUnits
    );
    this.pivotUnsafePositionalMin = pivotUnsafePositionalLimitsUnits.convertX(
        pivotUnsafePositionalMin, this.pivotUnsafePositionalLimitsUnits
    );
    this.breakBeamChannel = breakBeamChannel;
  }
}
