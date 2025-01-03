// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.complex.AimAllAtSpeakerCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotAltModes;
import frc.robot.utils.LoopTimer;
import frc.robot.utils.TagalongChoreoFollower;
import java.nio.file.Files;
import java.nio.file.Paths;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedSystemStats;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command _autonomousCommand;

  private RobotContainer _robotContainer;
  private CommandScheduler _scheduler;

  public Robot() {
    super(Constants.LOOP_PERIOD_S);
  }

  @Override
  public void robotInit() {
    // Ensure analog sensors don't error on boot
    LoggedSystemStats.getInputs().userVoltage5v = 5.0;
    _scheduler = CommandScheduler.getInstance();

    LoopTimer.markLoopStart();
    _robotContainer = new RobotContainer();
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    // Set up data receivers & replay source
    if (!Robot.isReal()) {
      // Running a physics simulator, log to NT
      // Logger.addDataReceiver(new NT4Publisher());
    } else if (RobotAltModes.isTestMode) {
      // Running test mode, log to roborio file
      Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs/"));
      // Logger.addDataReceiver(new NT4Publisher());
    } else { // REAL ROBOT
      // Running on a real robot, log to a USB stick
      if (!Files.exists(Paths.get("/U"))) {
        Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs/"));
      } else {
        Logger.addDataReceiver(new WPILOGWriter("/U"));
      }
      // Logger.addDataReceiver(new NT4Publisher());
    }

    if (RobotAltModes.isReplayMode) {
      // Replaying a log, set up replay source
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog();
      Logger.setReplaySource(new WPILOGReader(logPath));
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    }

    // Start AdvantageKit logger
    Logger.start();

    _robotContainer.robotInit();
  }

  @Override
  public void robotPeriodic() {
    _scheduler.run();
    _robotContainer.periodic();
    if (RobotAltModes.isLoopTiming) {
      System.out.println();
    }
  }

  @Override
  public void disabledInit() {
    _robotContainer.onDisable();
  }

  @Override
  public void disabledPeriodic() {
    _robotContainer.disabledPeriodic();
  }

  @Override
  public void disabledExit() {
    _robotContainer.onEnable();
  }

  @Override
  public void autonomousInit() {
    // AimAllAtSpeakerCommand.setChoreoAimDriver(true);
    // TagalongChoreoFollower.setAimingMode(true);
    _robotContainer.configFMSData();
    _robotContainer.autonomousInit();
    _autonomousCommand = _robotContainer._curAutoSelected.builtCommand;

    if (_autonomousCommand != null) {
      _autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    _robotContainer.log();
  }

  @Override
  public void autonomousExit() {
    AimAllAtSpeakerCommand.setChoreoAimDriver(false);
  }

  @Override
  public void teleopInit() {
    _robotContainer.configFMSData();
    _robotContainer.teleopInit();
    if (_autonomousCommand != null) {
      _autonomousCommand.cancel();
      _autonomousCommand = null;
    }
  }

  @Override
  public void teleopPeriodic() {
    _robotContainer.log();
    _robotContainer.teleopPeriodic();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  public void simulationInit() {
    _robotContainer.simulationInit();
  }

  public void simulationPeriodic() {
    _robotContainer.simulationPeriodic();
  }

  @Override
  public void testPeriodic() {
    System.out.println("Test Periodic hello");
    System.out.println(
        _robotContainer._shooter.getPivot().getPrimaryMotor().setPosition(9999999).toString()
    );

    System.out.println(_robotContainer._shooter.getPivot().getPrimaryMotor().get());
  }

  @Override
  public void testExit() {}
}
