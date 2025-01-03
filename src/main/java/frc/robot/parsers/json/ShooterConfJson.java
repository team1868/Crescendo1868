package frc.robot.parsers.json;

import frc.robot.parsers.json.utils.*;

public class ShooterConfJson extends TagalongBaseJson {
  public String name;
  public String flywheelFile;
  public String pivotFile;
  public String rollerFile;
  public UnitJson pivotHeight;
  public String pivotConflict;
  public PositionalLimitsJson pivotUnsafePositionalLimits;
  public PolynomialJson targetPivotQuadratic = PolynomialJson.getDefaultRotation();
  public PolynomialJson minPivotQuadratic = PolynomialJson.getDefaultRotation();
  public PolynomialJson maxPivotQuadratic = PolynomialJson.getDefaultRotation();
  public LinearizedJson targetPivotLinearized;
  public LinearizedJson minPivotLinearized;
  public LinearizedJson maxPivotLinearized;
  public LinearizedJson launchRPSLinearized;
  public int breakBeamChannel;
}
