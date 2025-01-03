package frc.robot.parsers;

import com.fasterxml.jackson.databind.ObjectMapper;
import frc.robot.parsers.json.utils.FlywheelConfJson;
import frc.robot.utils.FileUtils;
import java.io.File;

public class FlywheelParser {
  public FlywheelConfJson flywheelConf;

  public FlywheelParser(File dir, String filename) {
    try {
      File flywheelFile = new File(dir, filename);
      FileUtils.checkForFile(flywheelFile);
      flywheelConf = new ObjectMapper().readValue(flywheelFile, FlywheelConfJson.class);

    } catch (Exception err) {
      System.err.println(err);
      System.exit(1);
    }
  }
}
