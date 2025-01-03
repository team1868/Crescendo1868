package frc.robot.parsers;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.parsers.json.IntakeConfJson;
import frc.robot.utils.FileUtils;
import java.io.File;

public class IntakeParser {
  public IntakeConfJson intakeConf;
  public PivotParser pivotParser;
  public RollerParser rollerParser;

  public IntakeParser(File dir, String filename) {
    try {
      File intakeFile = new File(dir, filename);
      FileUtils.checkForFile(intakeFile);
      intakeConf = new ObjectMapper().readValue(intakeFile, IntakeConfJson.class);

      pivotParser = new PivotParser(
          new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/configs/intake"),
          intakeConf.pivotFile
      );

      rollerParser = new RollerParser(
          new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/configs/intake"),
          intakeConf.rollerFile
      );

    } catch (Exception err) {
      System.err.println(err);
      System.exit(1);
    }
  }
}
