package frc.robot.parsers;

import com.fasterxml.jackson.databind.ObjectMapper;
import frc.robot.parsers.json.utils.ElevatorConfJson;
import frc.robot.utils.FileUtils;
import java.io.File;

public class ElevatorParser {
  public ElevatorConfJson elevatorConf;

  public ElevatorParser(File dir, String filename) {
    try {
      File elevatorFile = new File(dir, filename);
      FileUtils.checkForFile(elevatorFile);
      elevatorConf = new ObjectMapper().readValue(elevatorFile, ElevatorConfJson.class);

    } catch (Exception err) {
      System.err.println(err);
      System.exit(1);
    }
  }
}
