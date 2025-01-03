package frc.robot.parsers;

import com.fasterxml.jackson.databind.ObjectMapper;
import frc.robot.parsers.json.utils.PivotConfJson;
import frc.robot.utils.FileUtils;
import java.io.File;

public class PivotParser {
  public PivotConfJson pivotConf;
  public PivotParser(File dir, String filename) {
    try {
      File pivotFile = new File(dir, filename);
      FileUtils.checkForFile(pivotFile);
      pivotConf = new ObjectMapper().readValue(pivotFile, PivotConfJson.class);

    } catch (Exception err) {
      System.err.println(err);
      System.exit(1);
    }
  }
}
