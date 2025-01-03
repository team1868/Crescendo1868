package frc.robot.parsers.json.utils;

import frc.robot.utils.LinearizedLookupTable;

public class LinearizedJson {
  public String unitX;
  public String unitY;
  public double[][] points;
  private double[] xVal;
  private double[] yVal;
  private LinearizedLookupTable linearized = null;

  private void separateXY() {
    xVal = new double[points.length];
    yVal = new double[points.length];
    for (int i = 0; i < points.length; i++) {
      xVal[i] = points[i][0];
      yVal[i] = points[i][1];
    }
  }

  public double getDegOutput(double x) {
    if (linearized == null) {
      separateXY();
      linearized = new LinearizedLookupTable(xVal, yVal);
    }

    return linearized.lookup(x);
  }

  public double getRPSOutput(double x) {
    if (linearized == null) {
      separateXY();
      linearized = new LinearizedLookupTable(xVal, yVal);
    }

    return linearized.lookup(x);
  }
}
