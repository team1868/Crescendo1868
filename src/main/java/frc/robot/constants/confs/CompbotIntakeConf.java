package frc.robot.constants.confs;

import tagalong.subsystems.micro.confs.PivotConf;
import tagalong.subsystems.micro.confs.RollerConf;
import tagalong.units.DistanceUnits;

public class CompbotIntakeConf extends BaseIntakeConf {
  public static final PivotConf pivotConf = CompbotIntakePivotConf.construct();
  public static final RollerConf rollerConf = CompbotIntakeRollerConf.construct();
  public static final String pivotConflict = "shooter";

  public static final DistanceUnits pivotUnsafePositionalLimitsUnits = DistanceUnits.DEGREE;
  public static final double pivotUnsafePositionalMin = 22.0;
  public static final double pivotUnsafePositionalMax = 106.79;

  public static final int breakBeamChannel = 1;

  public static CompbotIntakeConf construct() {
    return new CompbotIntakeConf(
        pivotConf,
        rollerConf,
        pivotConflict,
        pivotUnsafePositionalLimitsUnits,
        pivotUnsafePositionalMin,
        pivotUnsafePositionalMax,
        breakBeamChannel
    );
  }

  public CompbotIntakeConf(
      PivotConf pivotConf,
      RollerConf rollerConf,
      String pivotConflict,
      DistanceUnits pivotUnsafePositionalLimitsUnits,
      double pivotUnsafePositionalMin,
      double pivotUnsafePositionalMax,
      int breakBeamChannel
  ) {
    super(
        pivotConf,
        rollerConf,
        pivotConflict,
        pivotUnsafePositionalLimitsUnits,
        pivotUnsafePositionalMin,
        pivotUnsafePositionalMax,
        breakBeamChannel
    );
  }
}
