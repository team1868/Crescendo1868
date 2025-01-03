package frc.robot.parsers;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.parsers.json.ClimberConfJson;
import frc.robot.utils.FileUtils;
import java.io.File;

public class ClimberParser {
  public ClimberConfJson climberConf;
  public ElevatorParser elevatorParser;
  public RollerParser rollerParser;
  public ElevatorParser hookParser;

  public ClimberParser(File dir, String filename) {
    try {
      File climberFile = new File(dir, filename);
      FileUtils.checkForFile(climberFile);
      climberConf = new ObjectMapper().readValue(climberFile, ClimberConfJson.class);

      elevatorParser = new ElevatorParser(
          new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/configs/climber"),
          climberConf.elevatorFile
      );
      rollerParser = new RollerParser(
          new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/configs/climber"),
          climberConf.rollerFile
      );
      hookParser = new ElevatorParser(
          new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/configs/climber"),
          climberConf.hooksFile
      );

    } catch (Exception err) {
      System.err.println(err);
      System.exit(1);
    }
  }
}
