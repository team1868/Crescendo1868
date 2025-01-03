package frc.robot.parsers;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.parsers.json.ShooterConfJson;
import frc.robot.utils.FileUtils;
import java.io.File;

public class ShooterParser {
  public ShooterConfJson shooterConf;
  public FlywheelParser flywheelParser;
  public PivotParser pivotParser;
  public RollerParser rollerParser;

  public ShooterParser(File dir, String filename) {
    try {
      File shooterFile = new File(dir, filename);
      FileUtils.checkForFile(shooterFile);
      shooterConf = new ObjectMapper().readValue(shooterFile, ShooterConfJson.class);

      flywheelParser = new FlywheelParser(
          new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/configs/shooter"),
          shooterConf.flywheelFile
      );

      pivotParser = new PivotParser(
          new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/configs/shooter"),
          shooterConf.pivotFile
      );

      rollerParser = new RollerParser(
          new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/configs/shooter"),
          shooterConf.rollerFile
      );

    } catch (Exception err) {
      System.err.println(err);
      System.exit(1);
    }
  }
}
