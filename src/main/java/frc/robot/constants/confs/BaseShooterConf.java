
package frc.robot.constants.confs;

import frc.robot.subsystems.minor.confs.TagalongBaseFlywheelConf;
import tagalong.subsystems.micro.confs.PivotConf;
import tagalong.subsystems.micro.confs.RollerConf;
import tagalong.units.DistanceUnits;

public class BaseShooterConf {
  public final DistanceUnits pivotUnsafeLimitsUnits = DistanceUnits.ROTATION;
  public final DistanceUnits pivotQuadraticUnits = DistanceUnits.ROTATION;
  public final DistanceUnits pivotLinearizedXUnits = DistanceUnits.METER;
  public final DistanceUnits pivotLinearizedYUnits = DistanceUnits.ROTATION;
  public String name;
  public TagalongBaseFlywheelConf flywheelConf;
  public PivotConf pivotConf;
  public RollerConf rollerConf;
  public double pivotHeight;
  public String pivotConflict;
  public double pivotUnsafeMin;
  public double pivotUnsafeMax;

  public double[] targetPivotQuadratic;
  public double[] minPivotQuadratic;
  public double[] maxPivotQuadratic;
  public double[][] targetPivotLinearized;
  public double[][] maxPivotLinearized;
  public double[][] minPivotLinearized;
  public double[][] launchRPSLinearized;
  public int breakBeamChannel;

  public BaseShooterConf(
      String name,
      TagalongBaseFlywheelConf flywheelConf,
      PivotConf pivotConf,
      RollerConf rollerConf,
      double pivotHeight,
      String pivotConflict,
      DistanceUnits pivotUnsafeLimitsUnits,
      double pivotUnsafeMinRot,
      double pivotUnsafeMaxRot,
      DistanceUnits pivotQuadraticUnits,
      double[] targetPivotQuadratic,
      double[] minPivotQuadratic,
      double[] maxPivotQuadratic,
      DistanceUnits pivotLinearizedXUnits,
      DistanceUnits pivotLinearizedYUnits,
      double[][] targetPivotLinearized,
      double[][] maxPivotLinearized,
      double[][] minPivotLinearized,
      double[][] launchRPSLinearized,
      int breakBeamChannel
  ) {
    this.name = name;
    this.flywheelConf = flywheelConf;
    this.pivotConf = pivotConf;
    this.rollerConf = rollerConf;
    this.pivotHeight = pivotHeight;
    this.pivotConflict = pivotConflict;
    this.pivotUnsafeMin =
        pivotUnsafeLimitsUnits.convertX(pivotUnsafeMinRot, this.pivotUnsafeLimitsUnits);
    this.pivotUnsafeMax =
        pivotUnsafeLimitsUnits.convertX(pivotUnsafeMaxRot, this.pivotUnsafeLimitsUnits);
    ;
    this.targetPivotQuadratic = targetPivotQuadratic;
    this.minPivotQuadratic = minPivotQuadratic;
    this.maxPivotQuadratic = maxPivotQuadratic;
    this.targetPivotLinearized = targetPivotLinearized;
    this.maxPivotLinearized = maxPivotLinearized;
    this.minPivotLinearized = minPivotLinearized;
    this.launchRPSLinearized = launchRPSLinearized;
    this.breakBeamChannel = breakBeamChannel;
  }
}
