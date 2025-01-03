package frc.robot.parsers.json.utils;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class Rotation3dJson {
  public String unit;
  public double yaw;
  public double pitch;
  public double roll;
  private Rotation3d rotation = null;

  public Rotation3d getDistRotation() {
    if (rotation == null) {
      if (unit.equalsIgnoreCase("degree") || unit.equalsIgnoreCase("deg")) {
        rotation = new Rotation3d(
            Units.degreesToRadians(roll), Units.degreesToRadians(pitch), Units.degreesToRadians(yaw)
        );
      } else if (unit.equalsIgnoreCase("radian") || unit.equalsIgnoreCase("rad")) {
        rotation = new Rotation3d(roll, pitch, yaw);
      } else if (unit.equalsIgnoreCase("rotation") || unit.equalsIgnoreCase("rot")) {
        rotation = new Rotation3d(roll * 2.0 * Math.PI, pitch * 2.0 * Math.PI, yaw * 2.0 * Math.PI);
      } else {
        System.err.println("Incompatible dist types: expected rotation but got " + unit);
        System.exit(1);
      }
    }
    return rotation;
  }

  public Rotation3d getVeloRotation() {
    if (rotation == null) {
      if (unit.equalsIgnoreCase("DPS") || unit.equalsIgnoreCase("DegPS")) {
        rotation = new Rotation3d(
            Units.degreesToRadians(roll), Units.degreesToRadians(pitch), Units.degreesToRadians(yaw)
        );
      } else if (unit.equalsIgnoreCase("RadPS")) {
        rotation = new Rotation3d(roll, pitch, yaw);
      } else if (unit.equalsIgnoreCase("RPS") || unit.equalsIgnoreCase("RotPS")) {
        rotation = new Rotation3d(roll * 2.0 * Math.PI, pitch * 2.0 * Math.PI, yaw * 2.0 * Math.PI);
      } else {
        System.err.println("Incompatible velocity types: expected rotation but got " + unit);
        System.exit(1);
      }
    }
    return rotation;
  }

  public Rotation3d getAccelRotation() {
    if (rotation == null) {
      if (unit.equalsIgnoreCase("degps2") || unit.equalsIgnoreCase("dps2")) {
        rotation = new Rotation3d(
            Units.degreesToRadians(roll), Units.degreesToRadians(pitch), Units.degreesToRadians(yaw)
        );
      } else if (unit.equalsIgnoreCase("radps2")) {
        rotation = new Rotation3d(roll, pitch, yaw);
      } else if (unit.equalsIgnoreCase("rotps2")) {
        rotation = new Rotation3d(roll * 2.0 * Math.PI, pitch * 2.0 * Math.PI, yaw * 2.0 * Math.PI);
      } else {
        System.err.println("Incompatible acceleration types: expected rotation but got " + unit);
        System.exit(1);
      }
    }
    return rotation;
  }
}
