package frc.robot.parsers.json.utils;

import java.util.ArrayList;

public class ValidTagsJson {
  public ValidTagJson[] tags;

  // potentially pre compute
  public int[][][][] overlaps = new int[14][15][16][];

  public int[] calculateOverlap(int tagA, int tagB, int tagC) {
    int min = Math.min(tagA, Math.min(tagB, tagC));
    int max = Math.max(tagA, Math.max(tagB, tagC));
    int mid = tagA + tagB + tagC - min - max;
    if (overlaps[min][mid][max] == null) {
      var overlap = new ArrayList<Integer>();
      // only if tag is valid set
      if (!tags[tagA].isInvalid(tagB) && !tags[tagA].isInvalid(tagC)
          && !tags[tagB].isInvalid(tagC)) {
        int a = 0;
        int b = 0;
        int c = 0;
        while (a < tags[tagA].value.length && b < tags[tagB].value.length
               && c < tags[tagC].value.length) {
          int tempI = tags[tagA].value[a];
          int tempJ = tags[tagB].value[b];
          int tempK = tags[tagC].value[c];
          if (tempI == tempJ && tempI == tempK) {
            overlap.add(tempI);
            a++;
            b++;
            c++;
          } else {
            int minIJK = Math.min(tempI, Math.min(tempI, tempK));
            if (a == minIJK)
              a++;
            if (b == minIJK)
              b++;
            if (c == minIJK)
              c++;
          }
        }
      }

      // populate array
      overlaps[min][mid][max] = new int[overlap.size()];
      for (int i = 0; i < overlap.size(); i++) {
        overlaps[min][mid][max][i] = overlap.get(i);
      }
    }
    return overlaps[min][mid][max];
  }
}
