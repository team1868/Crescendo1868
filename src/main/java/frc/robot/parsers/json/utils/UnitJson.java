package frc.robot.parsers.json.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class UnitJson {
  public static enum UnitCategories { ROTATIONAL, TRANSLATIONAL }
  public static enum UnitTypes {
    METERS(UnitCategories.TRANSLATIONAL),
    INCHES(UnitCategories.TRANSLATIONAL),
    DEGREES(UnitCategories.ROTATIONAL),
    RADIANS(UnitCategories.ROTATIONAL),
    ROTATIONS(UnitCategories.ROTATIONAL);

    public final UnitCategories _variant;
    UnitTypes(UnitCategories varient) {
      _variant = varient;
    }
  }

  public String unit;
  public double value;

  public static UnitJson defaultDegrees() {
    var unitJson = new UnitJson();
    unitJson.unit = "degree";
    unitJson.value = 0.0;
    return unitJson;
  }

  public static UnitJson defaultRotations() {
    var unitJson = new UnitJson();
    unitJson.unit = "degree";
    unitJson.value = 0.0;
    return unitJson;
  }

  public double getLengthM() {
    if (unit.equalsIgnoreCase("inch") || unit.equalsIgnoreCase("in")) {
      return Units.inchesToMeters(value);
    } else if (unit.equalsIgnoreCase("meter") || unit.equalsIgnoreCase("m")) {
      return value;
    } else {
      System.err.println("Incompatible length types: expected length but got " + unit);
      System.exit(1);
    }
    return -1;
  }

  public double getVeloM() {
    if (unit.equalsIgnoreCase("meterps") || unit.equalsIgnoreCase("mps")) {
      return value;
    } else {
      System.err.println("Incompatible velocity types: expected mps but got " + unit);
      System.exit(1);
    }
    return -1;
  }

  public double getAccelM() {
    if (unit.equalsIgnoreCase("meterps2") || unit.equalsIgnoreCase("mps2")) {
      return value;
    } else {
      System.err.println("Incompatible acceleration types: expected mps2 but got " + unit);
      System.exit(1);
    }
    return -1;
  }

  public double getMassKg() {
    if (unit.equalsIgnoreCase("lb") || unit.equalsIgnoreCase("pound")) {
      return Units.lbsToKilograms(value);
    } else if (unit.equalsIgnoreCase("kg") || unit.equalsIgnoreCase("kilogram")) {
      return value;
    } else {
      System.err.println("Incompatible length types: expected length but got " + unit);
      System.exit(1);
    }
    return -1;
  }

  public double getMassLb() {
    if (unit.equalsIgnoreCase("kg") || unit.equalsIgnoreCase("kilogram")) {
      return Units.kilogramsToLbs(value);
    } else if (unit.equalsIgnoreCase("lb") || unit.equalsIgnoreCase("pound")) {
      return value;
    } else {
      System.err.println("Incompatible length types: expected length but got " + unit);
      System.exit(1);
    }
    return -1;
  }

  public Rotation2d getDistRotation() {
    return getAngleRotation2d(unit, value);
  }

  public Rotation2d getVeloRotation() {
    return getVelocityRotation2d(unit, value);
  }

  public Rotation2d getAccelRotation() {
    return getAccelerationRotation2d(unit, value);
  }

  public static Rotation2d getAngleRotation2d(String unit, double value) {
    if (unit.equalsIgnoreCase("degree") || unit.equalsIgnoreCase("deg")) {
      return Rotation2d.fromDegrees(value);
    } else if (unit.equalsIgnoreCase("radian") || unit.equalsIgnoreCase("rad")) {
      return Rotation2d.fromRadians(value);
    } else if (unit.equalsIgnoreCase("rotation") || unit.equalsIgnoreCase("rot")) {
      return Rotation2d.fromRotations(value);
    } else {
      System.err.println("Incompatible dist types: expected rotation but got " + unit);
      System.exit(1);
      return null;
    }
  }

  public static Rotation2d getVelocityRotation2d(String unit, double value) {
    if (unit.equalsIgnoreCase("DPS") || unit.equalsIgnoreCase("DegPS")) {
      return Rotation2d.fromDegrees(value);
    } else if (unit.equalsIgnoreCase("RadPS")) {
      return Rotation2d.fromRadians(value);
    } else if (unit.equalsIgnoreCase("RPS") || unit.equalsIgnoreCase("RotPS")) {
      return Rotation2d.fromRotations(value);
    } else {
      System.err.println("Incompatible velocity types: expected rotation but got " + unit);
      System.exit(1);
      return null;
    }
  }

  public static Rotation2d getAccelerationRotation2d(String unit, double value) {
    if (unit.equalsIgnoreCase("degps2") || unit.equalsIgnoreCase("dps2")) {
      return Rotation2d.fromDegrees(value);
    } else if (unit.equalsIgnoreCase("radps2")) {
      return Rotation2d.fromRadians(value);
    } else if (unit.equalsIgnoreCase("rotps2")) {
      return Rotation2d.fromRotations(value);
    } else {
      System.err.println("Incompatible acceleration types: expected rotation but got " + unit);
      System.exit(1);
      return null;
    }
  }

  public static UnitTypes getUnitFromString(String unit) {
    if (unit.equalsIgnoreCase("meter") || unit.equalsIgnoreCase("m")) {
      return UnitTypes.METERS;
    } else if (unit.equalsIgnoreCase("inch") || unit.equalsIgnoreCase("in")) {
      return UnitTypes.INCHES;
    } else if (unit.equalsIgnoreCase("degree") || unit.equalsIgnoreCase("deg")) {
      return UnitTypes.DEGREES;
    } else if (unit.equalsIgnoreCase("radian") || unit.equalsIgnoreCase("rad")) {
      return UnitTypes.RADIANS;
    } else if (unit.equalsIgnoreCase("rotation") || unit.equalsIgnoreCase("rot")) {
      return UnitTypes.ROTATIONS;
    } else {
      System.err.println("Incompatible dist types: got " + unit);
      System.exit(1);
      return null;
    }
  }

  public static double convertRotational(double base, UnitTypes curOutUnit, UnitTypes outUnit) {
    if (curOutUnit == outUnit)
      return base;
    switch (outUnit) {
      case DEGREES:
        switch (curOutUnit) {
          case RADIANS:
            return Units.radiansToDegrees(base);
          case ROTATIONS:
            return Units.rotationsToDegrees(base);
          default:
            errorAndExit("length but expected rotation");
        }
        break;
      case RADIANS:
        switch (curOutUnit) {
          case DEGREES:
            return Units.degreesToRadians(base);
          case ROTATIONS:
            return Units.rotationsToRadians(base);
          default:
            errorAndExit("length but expected rotation");
        }
      case ROTATIONS:
        switch (curOutUnit) {
          case DEGREES:
            return Units.degreesToRotations(base);
          case RADIANS:
            return Units.radiansToRotations(base);
          default:
            errorAndExit("length but expected rotation");
        }
      default:
        errorAndExit("length but expected rotation");
    }
    return 0.0;
  }

  public static void convertRotational(double[] base, UnitTypes curOutUnit, UnitTypes outUnit) {
    if (curOutUnit == outUnit) {
      return;
    }
    switch (outUnit) {
      case DEGREES:
        switch (curOutUnit) {
          case RADIANS:
            for (int i = 0; i < base.length; i++) base[i] = Units.radiansToDegrees(base[i]);
            return;
          case ROTATIONS:
            for (int i = 0; i < base.length; i++) base[i] = Units.rotationsToDegrees(base[i]);
            return;
          default:
            errorAndExit("length but expected rotation");
        }
        break;
      case RADIANS:
        switch (curOutUnit) {
          case DEGREES:
            for (int i = 0; i < base.length; i++) base[i] = Units.degreesToRadians(base[i]);
            return;
          case ROTATIONS:
            for (int i = 0; i < base.length; i++) base[i] = Units.rotationsToRadians(base[i]);
            return;
          default:
            errorAndExit("length but expected rotation");
        }
      case ROTATIONS:
        switch (curOutUnit) {
          case DEGREES:
            for (int i = 0; i < base.length; i++) base[i] = Units.degreesToRotations(base[i]);
            return;
          case RADIANS:
            for (int i = 0; i < base.length; i++) base[i] = Units.radiansToRotations(base[i]);
            return;
          default:
            errorAndExit("length but expected rotation");
        }
      default:
        errorAndExit("length but expected rotation");
    }
  }

  public static double convertTranslational(double base, UnitTypes curOutUnit, UnitTypes outUnit) {
    if (curOutUnit == outUnit) {
      return base;
    }
    switch (outUnit) {
      case METERS:
        switch (curOutUnit) {
          case INCHES:
            return Units.inchesToMeters(base);
          default:
            errorAndExit("rotation but expected length");
        }
        break;
      case INCHES:
        switch (curOutUnit) {
          case METERS:
            return Units.metersToInches(base);
          default:
            errorAndExit("rotation but expected length");
        }
      default:
        errorAndExit("rotation but expected length");
    }
    return 0.0;
  }

  public static void convertTranslational(double base[], UnitTypes curOutUnit, UnitTypes outUnit) {
    if (curOutUnit == outUnit) {
      return;
    }
    switch (outUnit) {
      case METERS:
        switch (curOutUnit) {
          case INCHES:
            for (int i = 0; i < base.length; i++) base[i] = Units.inchesToMeters(base[i]);
            return;
          default:
            errorAndExit("rotation but expected length");
        }
        break;
      case INCHES:
        switch (curOutUnit) {
          case METERS:
            for (int i = 0; i < base.length; i++) base[i] = Units.metersToInches(base[i]);
            return;
          default:
            errorAndExit("rotation but expected length");
        }
      default:
        errorAndExit("rotation but expected length");
    }
  }

  private static void errorAndExit(String errorMessage) {
    System.err.println("Incompatible: got" + errorMessage);
    System.exit(1);
  }
}
