// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.utils;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import frc.robot.parsers.json.utils.swerve.SwerveModuleJson;
import frc.robot.parsers.json.utils.swerve.SwerveModuleTypeConfJson;
import java.util.function.Supplier;

public class ModuleIOTalonFX implements ModuleIO {
  private final SwerveModuleJson conf;
  private final SwerveModuleTypeConfJson type;

  private final TalonFX driveMotor;
  private final TalonFX steerMotor;
  private final Supplier<Rotation2d> absoluteEncoderSupplier;

  private final StatusSignal<Double> drivePositionRot;
  private final StatusSignal<Double> driveVelocityRPS;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrentAmps;

  private final StatusSignal<Double> steerPositionRot;
  private final StatusSignal<Double> steerVelocityRPS;
  private final StatusSignal<Double> steerAppliedVolts;
  private final StatusSignal<Double> steerCurrentAmps;

  public ModuleIOTalonFX(TagalongSwerveModuleBase module) {
    conf = module._conf;
    type = module._type;

    driveMotor = module.getDriveMotor();
    steerMotor = module.getSteerMotor();

    absoluteEncoderSupplier = module::getIOModuleAngle;

    drivePositionRot = driveMotor.getPosition().clone();
    driveVelocityRPS = driveMotor.getVelocity().clone();
    driveAppliedVolts = driveMotor.getMotorVoltage().clone();
    driveCurrentAmps = driveMotor.getStatorCurrent().clone();

    steerPositionRot = steerMotor.getPosition().clone();
    steerVelocityRPS = steerMotor.getVelocity().clone();
    steerAppliedVolts = steerMotor.getMotorVoltage().clone();
    steerCurrentAmps = steerMotor.getStatorCurrent().clone();

    // TODO: Test with and without, potentially conflicts with other can optimization functions
    // BaseStatusSignal.setUpdateFrequencyForAll(
    //     RobotAltModes.kOdometryFrequency, new BaseStatusSignal[] {drivePositionRot,
    //     steerPositionRot}
    // );

    // BaseStatusSignal.setUpdateFrequencyForAll(
    //     50.0,
    //     new BaseStatusSignal[] {
    //         driveVelocityRPS,
    //         driveAppliedVolts,
    //         driveCurrentAmps,
    //         steerVelocityRPS,
    //         steerAppliedVolts,
    //         steerCurrentAmps
    //     }
    // );
    // if (Constants.isCANEncoder) {
    //   BaseStatusSignal.setUpdateFrequencyForAll(
    //       50.0, new BaseStatusSignal[] {steerAbsolutePosition}
    //   );
    // }
    // driveMotor.optimizeBusUtilization();
    // steerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // inputs.connected = BaseStatusSignal
    //                        .refreshAll(new BaseStatusSignal[] {
    //                            drivePositionRot,
    //                            driveVelocityRPS,
    //                            driveAppliedVolts,
    //                            driveCurrentAmps,
    //                            steerPositionRot,
    //                            steerVelocityRPS,
    //                            steerAppliedVolts,
    //                            steerCurrentAmps
    //                        })
    //                        .equals(StatusCode.OK);

    inputs.drivePositionRad = Units.rotationsToRadians(drivePositionRot.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocityRPS.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrentAmps.getValueAsDouble();

    inputs.steerAbsolutePosition = absoluteEncoderSupplier.get();

    inputs.steerPosition = Rotation2d.fromRotations(steerPositionRot.getValueAsDouble());
    inputs.steerVelocityRadPerSec = Units.rotationsToRadians(steerVelocityRPS.getValueAsDouble());
    inputs.steerAppliedVolts = steerAppliedVolts.getValueAsDouble();
    inputs.steerCurrentAmps = steerCurrentAmps.getValueAsDouble();
  }
}
