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
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.parsers.SwerveParser;
import frc.robot.subsystems.Drivetrain;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Drivetrain _drivetrain;
  public SwerveParser _conf;
  private final Pigeon2 pigeon;
  private final StatusSignal<Double> yawPosition, pitchPosition, rollPosition;
  private final StatusSignal<Double> yawVelocity, pitchVelocity, rollVelocity;

  public GyroIOPigeon2(Drivetrain drive) {
    _drivetrain = drive;
    _conf = drive._swerveParser;
    pigeon = drive.getGyro();

    yawPosition = pigeon.getYaw().clone();
    pitchPosition = pigeon.getPitch().clone();
    rollPosition = pigeon.getRoll().clone();

    yawVelocity = pigeon.getAngularVelocityZWorld().clone();
    pitchVelocity = pigeon.getAngularVelocityXWorld().clone();
    rollVelocity = pigeon.getAngularVelocityYWorld().clone();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    if (_conf != null) {
      // inputs.connected =
      //     BaseStatusSignal
      //         .refreshAll(
      //             yawPosition, pitchPosition, rollPosition, yawVelocity, pitchVelocity,
      //             rollVelocity
      //         )
      //         .equals(StatusCode.OK);
      inputs.yawPositionDeg = yawPosition.getValueAsDouble();
      inputs.pitchPositionDeg = pitchPosition.getValueAsDouble();
      inputs.rollPositionDeg = rollPosition.getValueAsDouble();
      inputs.yawVelocityDegPerSec = yawVelocity.getValueAsDouble();
      inputs.pitchVelocityDegPerSec = pitchVelocity.getValueAsDouble();
      inputs.rollVelocityDegPerSec = rollVelocity.getValueAsDouble();
    }
  }
}
