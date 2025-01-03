package frc.robot.parsers.json.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Translation2dJson {
  public String unit;
  public double x = 0;
  public double y = 0;
  private Translation2d translation = null;

  public Translation2d getLengthM() {
    if (translation == null) {
      if (unit.equalsIgnoreCase("inch") || unit.equalsIgnoreCase("in")) {
        translation = new Translation2d(Units.inchesToMeters(x), Units.inchesToMeters(y));
      } else if (unit.equalsIgnoreCase("meter") || unit.equalsIgnoreCase("m")) {
        translation = new Translation2d(x, y);
      } else {
        System.err.println("Incompatible length types: expected length but got " + unit);
        System.exit(1);
      }
    }
    return translation;
  }

  public Translation2d getVeloM() {
    if (translation == null) {
      if (unit.equalsIgnoreCase("meterps") || unit.equalsIgnoreCase("mps")) {
        translation = new Translation2d(x, y);
      } else {
        System.err.println("Incompatible velocity types: expected mps but got " + unit);
        System.exit(1);
      }
    }
    return translation;
  }

  public Translation2d getAccelM() {
    if (translation == null) {
      if (unit.equalsIgnoreCase("meterps2") || unit.equalsIgnoreCase("mps2")) {
        translation = new Translation2d(x, y);
      } else {
        System.err.println("Incompatible acceleration types: expected mps2 but got " + unit);
        System.exit(1);
      }
    }
    return translation;
  }
}
