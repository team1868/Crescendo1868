package frc.robot.parsers;

import com.fasterxml.jackson.databind.ObjectMapper;
import frc.robot.parsers.json.utils.RollerConfJson;
import frc.robot.utils.FileUtils;
import java.io.File;

public class RollerParser {
  public RollerConfJson rollerConf;

  public RollerParser(File dir, String filename) {
    try {
      File rollerFile = new File(dir, filename);
      FileUtils.checkForFile(rollerFile);
      rollerConf = new ObjectMapper().readValue(rollerFile, RollerConfJson.class);

    } catch (Exception err) {
      System.err.println(err);
      System.exit(1);
    }
  }
}
