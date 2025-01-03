package frc.robot.parsers.json;

import frc.robot.parsers.json.utils.*;

public class IntakeConfJson extends TagalongBaseJson {
  public String rollerFile;
  public String pivotFile;
  public String pivotConflict;
  public PositionalLimitsJson pivotUnsafePositionalLimits;
  public int breakBeamChannel;
}
