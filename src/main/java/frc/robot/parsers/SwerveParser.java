package frc.robot.parsers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.parsers.json.SwerveDriveJson;
import frc.robot.parsers.json.SwerveModuleControlJson;
import frc.robot.parsers.json.utils.swerve.SwerveModuleJson;
import frc.robot.parsers.json.utils.swerve.SwerveModuleTypeConfJson;
import frc.robot.utils.AnalogSwerveModule;
import frc.robot.utils.CancoderSwerveModule;
import frc.robot.utils.FileUtils;
import frc.robot.utils.FusedSwerveModule;
import frc.robot.utils.TagalongSwerveModuleBase;
import java.io.File;
/**
 * Helper class used to parse the JSON directory with specified configuration options.
 */
public class SwerveParser {
  // Parsed files
  public SwerveDriveJson swerveConf;
  public SwerveModuleControlJson moduleControlConf;
  public SwerveModuleTypeConfJson moduleTypeConf;
  public SwerveModuleJson[] moduleConfs;

  // Parsed or derived values
  public double DRIVE_MOTOR_FREE_RPM = 6080.0;

  public int _numModules;
  public double _theoreticalMaxWheelSpeedMPS;
  public double _theoreticalMaxTranslationSpeedMPS;
  public double _theoreticalMaxRotationalSpeedDPS;

  // local
  private TagalongSwerveModuleBase[] _modules = null;
  private Translation2d[] _swerveModuleLocations = null;

  public SwerveParser(File dir, String filename) {
    try {
      checkDirectoryStructure(dir);

      File drivetrainFile = new File(dir, filename);
      // Get the module files (and other associated config files for specified drivetrain)
      FileUtils.checkForFile(drivetrainFile);
      swerveConf = new ObjectMapper().readValue(drivetrainFile, SwerveDriveJson.class);

      File moduleControlFile =
          new File(dir, "configs/drivetrain/modules/" + swerveConf.moduleControl);
      FileUtils.checkForFile(moduleControlFile);
      moduleControlConf =
          new ObjectMapper().readValue(moduleControlFile, SwerveModuleControlJson.class);

      File moduleTypeFile = new File(
          dir, "configs/drivetrain/module_properties/" + moduleControlConf.moduleType + ".json"
      );
      FileUtils.checkForFile(moduleTypeFile);
      moduleTypeConf = new ObjectMapper().readValue(moduleTypeFile, SwerveModuleTypeConfJson.class);

      _numModules = swerveConf.moduleFiles.length;
      moduleConfs = new SwerveModuleJson[swerveConf.moduleFiles.length];
      boolean[] idUsage = new boolean[swerveConf.moduleFiles.length];
      for (int i = 0; i < swerveConf.moduleFiles.length; i++) {
        File moduleFile = new File(dir, "configs/drivetrain/modules/" + swerveConf.moduleFiles[i]);
        FileUtils.checkForFile(moduleFile);
        var module = new ObjectMapper().readValue(moduleFile, SwerveModuleJson.class);

        // Ensure module id uniqueness while sorting the modules by id number
        assert (!idUsage[module.id]);
        idUsage[module.id] = true;
        moduleConfs[module.id] = module;
      }

      _theoreticalMaxWheelSpeedMPS =
          DRIVE_MOTOR_FREE_RPM * moduleTypeConf.getMotorFreeRPMToSurfaceSpeed();

    } catch (Exception err) {
      System.err.println(err);
      System.exit(1);
    }
  }

  /**
   * Check directory structure.
   *
   * @param directory JSON Configuration Directory
   */
  private void checkDirectoryStructure(File directory) {
    File modules = new File(directory, "modules");
    assert modules.exists() && modules.isDirectory();

    File properties = new File(directory, "module_properties");
    assert properties.exists() && properties.isDirectory();
  }

  public TagalongSwerveModuleBase[] getSwerveModules() {
    if (_modules == null) {
      _modules = new TagalongSwerveModuleBase[moduleConfs.length];
      for (int i = 0; i < moduleConfs.length; i++) {
        var type = moduleConfs[i].getEncoderType();
        switch (type) {
          case ANALOG:
            _modules[moduleConfs[i].id] =
                new AnalogSwerveModule(moduleConfs[i].id, this, _theoreticalMaxWheelSpeedMPS);
            break;
          case CANCODER:
            _modules[moduleConfs[i].id] =
                new CancoderSwerveModule(moduleConfs[i].id, this, _theoreticalMaxWheelSpeedMPS);
            break;
          case FUSED_CANCODER:
            _modules[moduleConfs[i].id] =
                new FusedSwerveModule(moduleConfs[i].id, this, _theoreticalMaxWheelSpeedMPS);
            break;
          case NONE:
          default:
            _modules[moduleConfs[i].id] =
                new TagalongSwerveModuleBase(moduleConfs[i].id, this, _theoreticalMaxWheelSpeedMPS);
        }
      }
    }
    return _modules;
  }

  public Translation2d[] getSwerveModuleLocations() {
    if (_swerveModuleLocations == null) {
      _swerveModuleLocations = new Translation2d[moduleConfs.length];
      for (int i = 0; i < moduleConfs.length; i++) {
        _swerveModuleLocations[i] = moduleConfs[i].getModuleLocation();
      }
    }
    return _swerveModuleLocations;
  }

  public TalonFXConfiguration getSteerConfigurationWithFeedbackConfigs(SwerveModuleJson moduleConf
  ) {
    return moduleControlConf.getSteerConfiguration().withFeedback(moduleConf.getFeedbackConfigs(
        moduleTypeConf.getSteerRatio(), moduleTypeConf.getSteerEncoderRatio()
    ));
  }

  public static final class ElectricalConf {
    // /* Drive Motor Characterization Values */
    // public static final SimpleMotorFeedforward driveFeedforward = new
    // SimpleMotorFeedforward(0.2, 2.2201, 0.16343);

    // TODO: deal with these
    public static final double DRIVE_OPEN_LOOP_RAMP = 0.25;
    public static final double DRIVE_CLOSED_LOOP_RAMP = 0.0;
  }
}
