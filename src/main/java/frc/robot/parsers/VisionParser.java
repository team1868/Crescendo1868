package frc.robot.parsers;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.enums.CameraSets;
import frc.robot.parsers.json.CameraVisionConfJson;
import frc.robot.parsers.json.utils.CameraConfJson;
import frc.robot.parsers.json.utils.ValidTagsJson;
import frc.robot.utils.FileUtils;
import java.io.File;

public class VisionParser {
  public final CameraSets _setup;
  public CameraConfJson[] cameraConfs;
  public CameraVisionConfJson cameraVisionConf;
  public ValidTagsJson validTagMap;
  private final String validTagFilePath = "configs/vision/apriltag/validTags.json";

  // 0- right camera
  // 1- left camera
  // 2- back color camera

  public VisionParser(File dir, String filename, CameraSets setup) {
    _setup = setup;

    assert (_setup.ids.length <= MAX_NUM_CAMERAS);

    try {
      checkDirectoryStructure(dir);
      File visionFile = new File(dir, filename);
      FileUtils.checkForFile(visionFile);
      cameraVisionConf = new ObjectMapper().readValue(visionFile, CameraVisionConfJson.class);

      cameraConfs = new CameraConfJson[_setup.ids.length];
      for (int i = 0; i < _setup.ids.length; i++) {
        File cameraFile =
            new File(dir, "/configs/vision/cameras/" + cameraVisionConf.cameraFiles[_setup.ids[i]]);
        FileUtils.checkForFile(cameraFile);
        var camera = new ObjectMapper().readValue(cameraFile, CameraConfJson.class);
        cameraConfs[i] = camera;
      }

      File validTagFile = new File(dir, validTagFilePath);
      FileUtils.checkForFile(validTagFile);
      validTagMap = new ObjectMapper().readValue(validTagFile, ValidTagsJson.class);
    } catch (Exception err) {
      System.err.println(err);
      System.exit(1);
    }

    assert (_setup.ids.length <= cameraConfs.length);

    int[] ids = new int[cameraConfs.length];
    for (int i = 0; i < cameraConfs.length; i++) {
      assert (cameraConfs[i].id < MAX_NUM_CAMERAS);
      ids[i] = cameraConfs[i].id;
      this.cameraConfs[ids[i]] = cameraConfs[i];

      for (int j = 0; j < i; j++) {
        assert (ids[j] != ids[i]);
      }
    }
  }

  private void checkDirectoryStructure(File directory) {
    File cameras = new File(directory, "cameras");
    assert cameras.exists() && cameras.isDirectory();
  }

  public boolean isValidId(int id) {
    return id < MAX_NUM_CAMERAS && id >= 0 && cameraConfs[id] != null;
  }

  public Transform3d getRobotToCamera(int id) {
    return isValidId(id) ? cameraConfs[id].robotToCamera.getTransform3d() : new Transform3d();
  }
  public static final int MAX_NUM_CAMERAS = 3;

  public static final double VISION_ESTIMATOR_SCALE = 0.0;

  public static final double DEFAULT_X_DEV = 0.06;
  public static final double DEFAULT_Y_DEV = 0.24;
  public static final double DEFAULT_THETA_DEV = 180;

  public static final double SINGLE_TAG_MAX_DIST_M = 7;

  public static final int PIX_2D_TOLERANCE = 60;
}
