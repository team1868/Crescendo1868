package frc.robot.parsers;

import com.fasterxml.jackson.databind.ObjectMapper;
import frc.robot.parsers.json.utils.LedConfJson;
import frc.robot.utils.FileUtils;
import java.io.File;

public class LedParser {
  public LedConfJson _ledConf;

  public LedParser(File dir, String filename) {
    try {
      File LedFile = new File(dir, filename);
      FileUtils.checkForFile(LedFile);
      _ledConf = new ObjectMapper().readValue(LedFile, LedConfJson.class);
    } catch (Exception err) {
      System.err.println(err);
      System.exit(1);
    }
  }
}
