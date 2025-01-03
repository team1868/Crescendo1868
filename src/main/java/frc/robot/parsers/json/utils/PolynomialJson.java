package frc.robot.parsers.json.utils;

import frc.robot.parsers.json.utils.UnitJson.UnitCategories;
import frc.robot.parsers.json.utils.UnitJson.UnitTypes;

public class PolynomialJson {
  public String unit;
  public int order;
  public double[] coefficients;
  private double[] convertedCoefficient = null;
  private UnitTypes originalUnit;
  private UnitTypes curOutUnit;

  public static PolynomialJson getDefaultRotation() {
    var defaultQuadratic = getDefault();
    defaultQuadratic.unit = "degree";
    return defaultQuadratic;
  }

  public static PolynomialJson getDefaultTranslation() {
    var defaultQuadratic = getDefault();
    defaultQuadratic.unit = "meter";
    return defaultQuadratic;
  }

  private static PolynomialJson getDefault() {
    var defaultQuadratic = new PolynomialJson();
    defaultQuadratic.coefficients = new double[3];
    return defaultQuadratic;
  }

  public double getRadOutput(double x) {
    return getOutput(x, UnitTypes.RADIANS);
  }

  public double getDegOutput(double x) {
    return getOutput(x, UnitTypes.DEGREES);
  }

  public double getRotOutput(double x) {
    return getOutput(x, UnitTypes.ROTATIONS);
  }

  public double getInchOutput(double x) {
    return getOutput(x, UnitTypes.INCHES);
  }

  public double getMeterOutput(double x) {
    return getOutput(x, UnitTypes.METERS);
  }

  private void convertToDesired(UnitTypes outUnit) {
    if (originalUnit == null) {
      order = coefficients.length - 1;
      convertedCoefficient = new double[coefficients.length];
      for (int i = 0; i < coefficients.length; i++) {
        convertedCoefficient[i] = coefficients[i];
      }
      originalUnit = UnitJson.getUnitFromString(unit);
      curOutUnit = originalUnit;
    }
    if (curOutUnit != outUnit) {
      if (curOutUnit._variant != outUnit._variant) {
        System.err.println("Incompatible: got" + originalUnit + "and" + outUnit);
        System.exit(1);
      } else if (curOutUnit._variant == UnitCategories.ROTATIONAL) {
        UnitJson.convertRotational(convertedCoefficient, curOutUnit, outUnit);
        curOutUnit = outUnit;
      } else if (curOutUnit._variant == UnitCategories.TRANSLATIONAL) {
        UnitJson.convertTranslational(convertedCoefficient, curOutUnit, outUnit);
        curOutUnit = outUnit;
      } else {
        System.err.println("Incompatible: got" + originalUnit + "and" + outUnit);
        System.exit(1);
      }
    }
  }

  protected double getOutput(double x, UnitTypes outUnit) {
    if (outUnit != curOutUnit) {
      convertToDesired(outUnit);
    }
    double sum = 0;
    for (int i = 0; i < coefficients.length; i++) {
      sum += convertedCoefficient[i] * Math.pow(x, order - i);
    }
    return sum;
  }
}
