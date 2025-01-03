package frc.robot.parsers.json.utils;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class DefaultTolerancesJson {
  public String unit;
  public double lowerTolerance;
  public double upperTolerance;
  private Rotation2d lowerRotation2d = null;
  private Rotation2d upperRotation2d = null;

  public double getLowerToleranceM() {
    if (unit.equalsIgnoreCase("inch")) {
      return Units.inchesToMeters(upperTolerance);
    } else if (unit.equalsIgnoreCase("meter")) {
      return lowerTolerance;
    } else {
      System.err.println("Incompatible trapezoidal types: expected length but got " + unit);
      System.exit(1);
    }
    return 0.0;
  }

  public double getLowerToleranceRad() {
    if (lowerRotation2d == null) {
      lowerRotation2d = UnitJson.getAngleRotation2d(unit, lowerTolerance);
    }
    return lowerRotation2d.getRadians();
  }

  public double getLowerToleranceRot() {
    if (lowerRotation2d == null) {
      lowerRotation2d = UnitJson.getAngleRotation2d(unit, lowerTolerance);
    }
    return lowerRotation2d.getRotations();
  }

  public double getLowerToleranceDeg() {
    if (lowerRotation2d == null) {
      lowerRotation2d = UnitJson.getAngleRotation2d(unit, lowerTolerance);
    }
    return lowerRotation2d.getDegrees();
  }

  public double getUpperToleranceM() {
    if (unit.equalsIgnoreCase("inch")) {
      return Units.inchesToMeters(upperTolerance);
    } else if (unit.equalsIgnoreCase("meter")) {
      return upperTolerance;
    } else {
      System.err.println("Incompatible trapezoidal types: expected length but got " + unit);
      System.exit(1);
    }
    return 0.0;
  }

  public double getUpperToleranceRad() {
    if (upperRotation2d == null) {
      upperRotation2d = UnitJson.getAngleRotation2d(unit, upperTolerance);
    }
    return upperRotation2d.getRadians();
  }

  public double getUpperToleranceRot() {
    if (upperRotation2d == null) {
      upperRotation2d = UnitJson.getAngleRotation2d(unit, upperTolerance);
    }
    return upperRotation2d.getRotations();
  }

  public double getUpperToleranceDeg() {
    if (upperRotation2d == null) {
      upperRotation2d = UnitJson.getAngleRotation2d(unit, upperTolerance);
    }
    return upperRotation2d.getDegrees();
  }
}
