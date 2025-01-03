package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Ports;
import frc.robot.constants.Constants;

public class Controlboard {
  private static Controlboard _controlboard;
  public static CommandXboxController _xboxDrive =
      new CommandXboxController(Ports.DRIVER_XBOX_USB_PORT);

  public static final Field2d _field = new Field2d();
  private int _lastOperatorPOVPressed = -1;
  private Alliance _alliance;
  private static boolean _isRed;

  public Controlboard() {
    configShuffleboard();
  }

  public static Controlboard get() {
    if (_controlboard == null) {
      _controlboard = new Controlboard();
    }
    return _controlboard;
  }

  public void periodic() {
    updateShuffleboard();
  }

  protected void configShuffleboard() {}

  protected void updateShuffleboard() {}

  public void updateAlliance(Alliance alliance) {
    _alliance = alliance;
    _isRed = (_alliance == Alliance.Red);
    if (Constants.CRobot._cameraSets != null) {
      MultiCameraController._primaryCam = _isRed ? Constants.CRobot._cameraSets.redPrimary
                                                 : Constants.CRobot._cameraSets.bluePrimary;
      MultiCameraController._secondaryCam = _isRed ? Constants.CRobot._cameraSets.bluePrimary
                                                   : Constants.CRobot._cameraSets.redPrimary;
    }
  }

  public static boolean isRedAlliance() {
    return _isRed;
  }

  public Rotation2d allianceGyroAngle() {
    return _isRed ? Constants.Sensors.GYRO_ZERO_RED : Constants.Sensors.GYRO_ZERO_BLUE;
  }

  public double POVZeroOffsetDeg() {
    return _isRed ? Constants.Sensors.POV_ZERO_RED_DEG : Constants.Sensors.POV_ZERO_BLUE_DEG;
  }

  // Left is positive X in terms of field, negative so our controller aligns
  public double getDriveX() {
    return _isRed ? _xboxDrive.getLeftY() : -_xboxDrive.getLeftY();
  }

  public double getDriveY() {
    return _isRed ? _xboxDrive.getLeftX() : -_xboxDrive.getLeftX();
  }

  public double getRotX() {
    return -_xboxDrive.getRightX();
  }

  public double getRotY() {
    return _xboxDrive.getRightY();
  }

  public void driverRumble() {
    driverRumble(1.0);
  }

  public void driverRumble(double power) {
    _xboxDrive.getHID().setRumble(RumbleType.kBothRumble, power);
  }

  public void driverResetRumble() {
    _xboxDrive.getHID().setRumble(RumbleType.kBothRumble, 0.0);
  }

  public InstantCommand driverRumbleCommand() {
    return new InstantCommand(() -> driverRumble());
  }

  public InstantCommand driverResetRumbleCommand() {
    return new InstantCommand(() -> driverResetRumble());
  }

  public static Trigger driverZeroGyroButton() {
    return _xboxDrive.start();
  }

  public static Trigger driverGroundIntakeButton() {
    return _xboxDrive.leftBumper();
  }

  public static Trigger driverEjectButton() {
    return _xboxDrive.rightBumper();
  }

  public static Trigger driverScoreSpeakerSubwooferButton() {
    return _xboxDrive.x();
  }

  public static Trigger driverScoreSpeakerOnFlyButton() {
    return _xboxDrive.rightTrigger();
  }

  public static Trigger driverScoreAmpButton() {
    return _xboxDrive.leftTrigger();
  }

  public static Trigger driverClimbExtendButton() {
    return _xboxDrive.povUp();
  }

  public static Trigger driverClimbRetractButton() {
    return _xboxDrive.povDown();
  }

  public static Trigger driverSkipNoteButton() {
    return _xboxDrive.a();
  }

  public static Trigger driverLaunchAtAmpButton() {
    return _xboxDrive.b();
  }

  public static Trigger driverSourceIntakeButton() {
    return _xboxDrive.y();
  }
}
