
package frc.robot.constants.confs;

import frc.robot.subsystems.minor.confs.TagalongBaseFlywheelConf;
import tagalong.subsystems.micro.confs.PivotConf;
import tagalong.subsystems.micro.confs.RollerConf;
import tagalong.units.DistanceUnits;

public class CompbotShooterConf extends BaseShooterConf {
  public static final String name = "Shooter";
  public static final TagalongBaseFlywheelConf flywheelConf =
      CompbotShooterFlywheelConf.construct();
  public static final PivotConf pivotConf = CompbotShooterPivotConf.construct();
  public static final RollerConf rollerConf = CompbotShooterRollerConf.construct();
  public static final double pivotHeight = 20.75;
  public static final String pivotConflict = "intake";

  public static final DistanceUnits pivotUnsafeLimitsUnits = DistanceUnits.DEGREE;
  public static final double pivotUnsafeMinRot = -39.0;
  public static final double pivotUnsafeMaxRot = 17.0;

  public static final DistanceUnits pivotQuadraticUnits = DistanceUnits.DEGREE;
  public static final double[] targetPivotQuadratic = {0.2862, -4.571, 26.367, -69.884};
  public static final double[] minPivotQuadratic = {0.2879, -4.4831, 25.174, -65.903};
  public static final double[] maxPivotQuadratic = {0.2844, -4.6589, 27.56, -73.866};

  public static final DistanceUnits pivotLinearizedXUnits = DistanceUnits.METER;
  public static final DistanceUnits pivotLinearizedYUnits = DistanceUnits.DEGREE;
  public static final double[][] targetPivotLinearized = {
      {1.358899266, -46.83007813},
      {1.892298978, -38.23242188},
      {2.501898649, -30.5859375},
      {3.086098334, -25.33984375},
      {3.270070503, -24.726},
      {3.695698004, -22.1484375},
      {4.305297675, -18.67539063},
      {4.914897346, -18.72070313},
      {5.562596996, -16.55},
      {6.273796612, -14.15039063},
      {6.756396352, -14.00546875},
      {7.378696016, -14.62070313}};
  public static final double[][] maxPivotLinearized = {
      {1.358899266, -41.66015625},
      {1.892298978, -34.453125},
      {2.501898649, -27.7734375},
      {3.086098334, -23.37890625},
      {3.270070503, -22.477},
      {3.695698004, -20.390625},
      {4.305297675, -17.05078125},
      {4.914897346, -17.75390625},
      {5.562596996, -15.5},
      {6.273796612, -13.359375},
      {6.756396352, -13.7109375},
      {7.378696016, -14.3}};
  public static final double[][] minPivotLinearized = {
      {1.358899266, -52},
      {1.892298978, -42.01171875},
      {2.501898649, -33.3984375},
      {3.086098334, -27.80078125},
      {3.270070503, -26.975},
      {3.695698004, -23.90625},
      {4.305297675, -20.3},
      {4.914897346, -19.6875},
      {5.562596996, -17.6},
      {6.273796612, -14.94140625},
      {6.756396352, -14.3},
      {7.378696016, -14.94140625}};
  public static final double[][] launchRPSLinearized = {
      {1.0, 12.0},
      {2.0, 14.0},
      {5.0, 25.0},
      {6.5, 28.0},
      {8.0, 35.0},
      {10.0, 38.0},
      {12.5, 43.0},
      {13.25, 46.0}};
  public static final int breakBeamChannel = 0;

  public static CompbotShooterConf construct() {
    return new CompbotShooterConf(
        name,
        flywheelConf,
        pivotConf,
        rollerConf,
        pivotHeight,
        pivotConflict,
        pivotUnsafeLimitsUnits,
        pivotUnsafeMinRot,
        pivotUnsafeMaxRot,
        pivotQuadraticUnits,
        targetPivotQuadratic,
        minPivotQuadratic,
        maxPivotQuadratic,
        pivotLinearizedXUnits,
        pivotLinearizedYUnits,
        targetPivotLinearized,
        maxPivotLinearized,
        minPivotLinearized,
        launchRPSLinearized,
        breakBeamChannel
    );
  }

  public CompbotShooterConf(
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
    super(
        name,
        flywheelConf,
        pivotConf,
        rollerConf,
        pivotHeight,
        pivotConflict,
        pivotUnsafeLimitsUnits,
        pivotUnsafeMinRot,
        pivotUnsafeMaxRot,
        pivotQuadraticUnits,
        targetPivotQuadratic,
        minPivotQuadratic,
        maxPivotQuadratic,
        pivotLinearizedXUnits,
        pivotLinearizedYUnits,
        targetPivotLinearized,
        maxPivotLinearized,
        minPivotLinearized,
        launchRPSLinearized,
        breakBeamChannel
    );
  }
}
