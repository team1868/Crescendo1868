package frc.robot.parsers.json.utils;

public class TagalongBaseJson {
  protected double calculateRatio(int[][] gearPairs) {
    double ratio = 1.0;
    for (int i = 0; i < gearPairs.length; i++) {
      assert (gearPairs[i].length == 2);
      ratio = ratio * gearPairs[i][0] / gearPairs[i][1];
    }
    return ratio;
  }
}
