package frc.robot.parsers;

import com.fasterxml.jackson.databind.ObjectMapper;
import frc.robot.parsers.json.utils.RollerConfJson;
import frc.robot.parsers.json.utils.RollerSimConfJson;
import frc.robot.utils.FileUtils;
import java.io.File;

public class RollerParser {
  public RollerConfJson rollerConf;
  public RollerSimConfJson rollerSimConf;

  public RollerParser(File dir, String filename) {
    try {
      File rollerFile = new File(dir, filename);
      FileUtils.checkForFile(rollerFile);
      rollerConf = new ObjectMapper().readValue(rollerFile, RollerConfJson.class);

      if (rollerConf.rollerSimFile != null) {
        File rollerSimFile = new File(dir, rollerConf.rollerSimFile);
        FileUtils.checkForFile(rollerSimFile);
        rollerSimConf = new ObjectMapper().readValue(rollerSimFile, RollerSimConfJson.class);
      } else {
        rollerSimConf = new RollerSimConfJson();
        rollerSimConf.MOI = 0.02;
        rollerSimConf.numLigaments = 3;
        rollerSimConf.numMotors = 1;
      }
    } catch (Exception err) {
      System.err.println(err);
      System.exit(1);
    }
  }
}
