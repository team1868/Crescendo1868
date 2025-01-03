package frc.robot.parsers.json.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class PositionalLimitsJson {
  public String unit;
  public double min;
  public double max;
  private boolean hasParsedMin = false;
  private boolean hasParsedMax = false;
  private double parsedMin;
  private double parsedMax;

  private Rotation2d minRotation2d = null;
  private Rotation2d maxRotation2d = null;

  public double getMinM() {
    if (hasParsedMin) {
      return parsedMin;
    } else if (unit.equalsIgnoreCase("inch")) {
      parsedMin = Units.inchesToMeters(min);
    } else if (unit.equalsIgnoreCase("meter")) {
      parsedMin = min;
    } else {
      System.err.println("Incompatible trapezoidal types: expected length but got " + unit);
      System.exit(1);
    }
    hasParsedMin = true;
    return parsedMin;
  }

  public double getMaxM() {
    if (hasParsedMax) {
      return parsedMax;
    } else if (unit.equalsIgnoreCase("inch")) {
      parsedMax = Units.inchesToMeters(max);
    } else if (unit.equalsIgnoreCase("meter")) {
      parsedMax = max;
    } else {
      System.err.println("Incompatible trapezoidal types: expected length but got " + unit);
      System.exit(1);
    }
    hasParsedMax = true;
    return parsedMax;
  }

  public double getMinDeg() {
    if (minRotation2d == null) {
      minRotation2d = UnitJson.getAngleRotation2d(unit, min);
    }
    return minRotation2d.getDegrees();
  }

  public double getMaxDeg() {
    if (maxRotation2d == null) {
      maxRotation2d = UnitJson.getAngleRotation2d(unit, max);
    }
    return maxRotation2d.getDegrees();
  }

  public double getMinRot() {
    if (minRotation2d == null) {
      minRotation2d = UnitJson.getAngleRotation2d(unit, min);
    }
    return minRotation2d.getRotations();
  }

  public double getMaxRot() {
    if (maxRotation2d == null) {
      maxRotation2d = UnitJson.getAngleRotation2d(unit, max);
    }
    return maxRotation2d.getRotations();
  }

  public double getMinRad() {
    if (minRotation2d == null) {
      minRotation2d = UnitJson.getAngleRotation2d(unit, min);
    }
    return minRotation2d.getRadians();
  }

  public double getMaxRad() {
    if (maxRotation2d == null) {
      maxRotation2d = UnitJson.getAngleRotation2d(unit, max);
    }
    return maxRotation2d.getRadians();
  }
}
