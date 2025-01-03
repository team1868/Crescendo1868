package frc.robot.parsers.json.utils.swerve;

import frc.robot.parsers.json.utils.TagalongBaseJson;
import frc.robot.parsers.json.utils.UnitJson;

public class SwerveModuleTypeConfJson extends TagalongBaseJson {
  public UnitJson wheelDiameter;

  public int[][] steerRatio;
  public int[][] steerEncoderRatio;
  public int[][] driveRatio;
  private double calculatedSteerRatio = -1;
  private double calculatedSteerEncoderRatio = -1;
  private double calculatedDriveRatio = -1;
  private double calculatedSurfaceSpeedRatio = -1;
  private double calculatedWheelCircumferenceM = -1;

  public double getMotorFreeRPMToSurfaceSpeed() {
    if (calculatedSurfaceSpeedRatio <= 0) {
      calculatedSurfaceSpeedRatio = (1.0 / 60.0) * getWheelCircumferenceM() / getDriveRatio();
    }
    return calculatedSurfaceSpeedRatio;
  }

  public double getWheelCircumferenceM() {
    if (calculatedWheelCircumferenceM <= 0) {
      calculatedWheelCircumferenceM = Math.PI * wheelDiameter.getLengthM();
    }
    return calculatedWheelCircumferenceM;
  }

  public double getSteerRatio() {
    if (calculatedSteerRatio <= 0) {
      calculatedSteerRatio = calculateRatio(steerRatio);
    }
    return calculatedSteerRatio;
  }

  public double getSteerEncoderRatio() {
    if (calculatedSteerEncoderRatio <= 0) {
      calculatedSteerEncoderRatio = calculateRatio(steerEncoderRatio);
    }
    return calculatedSteerEncoderRatio;
  }

  public double getDriveRatio() {
    if (calculatedDriveRatio <= 0) {
      calculatedDriveRatio = calculateRatio(driveRatio);
    }
    return calculatedDriveRatio;
  }
}
