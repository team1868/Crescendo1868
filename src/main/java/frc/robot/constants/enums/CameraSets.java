package frc.robot.constants.enums;

import java.util.Arrays;

public enum CameraSets {
  NO_CAMERAS(new int[] {}, -1, -1),
  CAMERA_0(new int[] {0}, 0, 0),
  CAMERA_1(new int[] {1}, 1, 1),
  CAMERA_2(new int[] {2}, 2, 2),
  BOTH_MONO_CAMERAS(new int[] {0, 1}, 0, 1),
  CAM_0_AND_CAM_2(new int[] {0, 2}, 0, 0),
  CAM_1_AND_CAM_2(new int[] {1, 2}, 1, 1),
  THREE_CAMERAS(new int[] {0, 1, 2}, 0, 1);

  public final int[] ids;
  public final int redPrimary;
  public final int bluePrimary;
  private boolean[] compatibleCameraSet;

  CameraSets(int[] cameraIDs, int redPrimary, int bluePrimary) {
    ids = cameraIDs;
    Arrays.sort(ids);
    this.redPrimary = redPrimary;
    this.bluePrimary = bluePrimary;
  }

  public boolean isCompatible(CameraSets set) {
    if (compatibleCameraSet == null) {
      CameraSets[] sets = CameraSets.values();
      compatibleCameraSet = new boolean[sets.length];

      // Caching this list is much better than doing it each time
      for (int i = 0; i < compatibleCameraSet.length; i++) {
        compatibleCameraSet[i] = idSubset(sets[i]);
      }
    }

    return compatibleCameraSet[set.ordinal()];
  }

  // Find if passed sets ids are a subset of current camera set's ids
  private boolean idSubset(CameraSets subset) {
    // for each id, if the id is not found in the ids array return false
    for (int i = 0; i < subset.ids.length; i++) {
      if (Arrays.binarySearch(ids, subset.ids[i]) == 0)
        return false;
    }

    // set is a subset of ids
    return true;
  }

  public boolean containsID(int id) {
    if (ids.length == 0) {
      return false;
    }
    return Arrays.binarySearch(ids, id) != 0;
  }

  public boolean hasCamera0() {
    return this == CAMERA_0 || this == BOTH_MONO_CAMERAS || this == CAM_0_AND_CAM_2
        || this == THREE_CAMERAS;
  }
  public boolean hasCamera1() {
    return this == CAMERA_1 || this == BOTH_MONO_CAMERAS || this == CAM_1_AND_CAM_2
        || this == THREE_CAMERAS;
  }
  public boolean hasCamera2() {
    return this == CAMERA_2 || this == CAM_0_AND_CAM_2 || this == CAM_1_AND_CAM_2
        || this == THREE_CAMERAS;
  }
}
