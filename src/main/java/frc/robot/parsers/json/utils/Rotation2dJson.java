package frc.robot.parsers.json.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Rotation2dJson {
  public String unit;
  public double angle;
  private Rotation2d rotation = null;

  public Rotation2d getDistRotation() {
    if (rotation == null) {
      if (unit.equalsIgnoreCase("degree") || unit.equalsIgnoreCase("deg")) {
        rotation = Rotation2d.fromDegrees(angle);
      } else if (unit.equalsIgnoreCase("radian") || unit.equalsIgnoreCase("rad")) {
        rotation = Rotation2d.fromRadians(angle);
      } else if (unit.equalsIgnoreCase("rotation") || unit.equalsIgnoreCase("rot")) {
        rotation = Rotation2d.fromRotations(angle * 2.0 * Math.PI);
      } else {
        System.err.println("Incompatible dist types: expected rotation but got " + unit);
        System.exit(1);
      }
    }
    return rotation;
  }

  public Rotation2d getVeloRotation() {
    if (rotation == null) {
      if (unit.equalsIgnoreCase("DPS") || unit.equalsIgnoreCase("DegPS")) {
        rotation = Rotation2d.fromDegrees(Units.degreesToRadians(angle));
      } else if (unit.equalsIgnoreCase("RadPS")) {
        rotation = Rotation2d.fromRadians(angle);
      } else if (unit.equalsIgnoreCase("RPS") || unit.equalsIgnoreCase("RotPS")) {
        rotation = Rotation2d.fromRotations(angle * 2.0 * Math.PI);
      } else {
        System.err.println("Incompatible velocity types: expected rotation but got " + unit);
        System.exit(1);
      }
    }
    return rotation;
  }

  public Rotation2d getAccelRotation() {
    if (rotation == null) {
      if (unit.equalsIgnoreCase("degps2") || unit.equalsIgnoreCase("dps2")) {
        rotation = Rotation2d.fromDegrees(Units.degreesToRadians(angle));
      } else if (unit.equalsIgnoreCase("radps2")) {
        rotation = Rotation2d.fromRadians(angle);
      } else if (unit.equalsIgnoreCase("rotps2")) {
        rotation = Rotation2d.fromRotations(angle * 2.0 * Math.PI);
      } else {
        System.err.println("Incompatible acceleration types: expected rotation but got " + unit);
        System.exit(1);
      }
    }
    return rotation;
  }
}
