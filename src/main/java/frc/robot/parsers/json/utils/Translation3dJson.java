package frc.robot.parsers.json.utils;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Translation3dJson {
  public String unit;
  public double x = 0;
  public double y = 0;
  public double z = 0;
  private Translation3d translation = null;

  public Translation3d getLengthM() {
    if (translation == null) {
      if (unit.equalsIgnoreCase("inch") || unit.equalsIgnoreCase("in")) {
        translation = new Translation3d(
            Units.inchesToMeters(x), Units.inchesToMeters(y), Units.inchesToMeters(z)
        );
      } else if (unit.equalsIgnoreCase("meter") || unit.equalsIgnoreCase("m")) {
        translation = new Translation3d(x, y, z);
      } else {
        System.err.println("Incompatible length types: expected length but got " + unit);
        System.exit(1);
      }
    }
    return translation;
  }

  public Translation3d getVeloM() {
    if (translation == null) {
      if (unit.equalsIgnoreCase("meterps") || unit.equalsIgnoreCase("mps")) {
        translation = new Translation3d(x, y, z);
      } else {
        System.err.println("Incompatible velocity types: expected mps but got " + unit);
        System.exit(1);
      }
    }
    return translation;
  }

  public Translation3d getAccelM() {
    if (translation == null) {
      if (unit.equalsIgnoreCase("meterps2") || unit.equalsIgnoreCase("mps2")) {
        translation = new Translation3d(x, y, z);
      } else {
        System.err.println("Incompatible acceleration types: expected mps2 but got " + unit);
        System.exit(1);
      }
    }
    return translation;
  }
}
