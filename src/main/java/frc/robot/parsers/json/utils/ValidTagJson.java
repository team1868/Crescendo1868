package frc.robot.parsers.json.utils;

import java.util.Arrays;

public class ValidTagJson {
  public int key;
  public int[] value = {};

  public boolean isInvalid(int key) {
    return Arrays.binarySearch(value, key) >= 0;
  }
}
