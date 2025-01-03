package frc.robot.utils;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.complex.AimAllAtSpeakerCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.enums.AutonomousActions;
import frc.robot.constants.enums.FieldVersions;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import java.util.HashMap;
import java.util.Map;

/** Utilities to load and follow ChoreoTrajectories */
public class TagalongChoreoFollower extends Command {
  public static final Controlboard _controlboard = Controlboard.get();
  private final Drivetrain _drivetrain;
  private final TagalongChoreoTrajectory _trajectory;
  private final PIDController _xController;
  private final PIDController _yController;
  private final PIDController _thetaController;
  private final Timer _timer = new Timer();
  private boolean _invertTrajectory;
  private boolean _invertTrajectoryAndNASA;
  private final ChoreoTrajectoryState _finalState;
  private final boolean _stopAtEnd;
  private Pose2d _currentPose;
  private Pose2d _finalPose;
  public static boolean _aimingMode;

  private boolean[] _setpointSeen;
  private Map<Integer, AutonomousActions> _commandMap;
  private Map<AutonomousActions, Command> _actionMap = null;
  private int _lastIndex;

  public static AimAllAtSpeakerCommand autoAimDriver;

  // TODO: change where this comes from
  private final double _xAdjust = 0.0;
  private final double _yAdjust = 0.0;

  public TagalongChoreoFollower(
      Drivetrain drivetrain,
      Shooter shooter,
      Intake intake,
      TagalongChoreoTrajectory trajectory,
      Map<Integer, AutonomousActions> commandMap
  ) {
    _drivetrain = drivetrain;
    _trajectory = trajectory;

    _commandMap = commandMap == null ? new HashMap<Integer, AutonomousActions>() : commandMap;

    // TODO: change where this comes from
    _xController = new PIDController(5.0, 0.0, 0.0);
    _yController = new PIDController(5.0, 0.0, 0.0);
    _thetaController = new PIDController(1.8, 0.0, 0.0);
    _thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Precompute final values
    _finalState = _trajectory.getFinalState();
    _stopAtEnd = _finalState.angularVelocity == 0.0 && _finalState.velocityX == 0.0
        && _finalState.velocityY == 0.0;
    _finalPose = _finalState.getPose();
    addRequirements(_drivetrain);
  }

  @Override
  public void initialize() {
    if (_actionMap == null) {
      _actionMap = AutonomousActions.getEventMap();
    }
    _invertTrajectory = Controlboard.isRedAlliance();
    _invertTrajectoryAndNASA = _invertTrajectory && (Constants.CField == FieldVersions.NASA_FIELD);
    _setpointSeen = new boolean[_trajectory.getNumSamples()];
    autoAimDriver.initialize();
    _lastIndex = 0;
    _timer.restart();
  }

  @Override
  public void execute() {
    double time = _timer.get();
    int ahead = _trajectory.getAheadStateIndex(_lastIndex, time);
    ChoreoTrajectoryState goal = _trajectory.sample(ahead, time, _invertTrajectory);
    _currentPose = _drivetrain.getPose();
    // System.out.println("mode: " + _aimingMode);
    if (!_aimingMode) {
      _drivetrain.robotCentricDrive(ChassisSpeeds.fromFieldRelativeSpeeds(
          goal.velocityX
              + _xController.calculate(
                  // _currentPose.getX(), _invertTrajectoryAndNASA ? goal.x : goal.x + _xAdjust
                  _currentPose.getX(),
                  goal.x + _xAdjust
              ),
          goal.velocityY + _yController.calculate(_currentPose.getY(), goal.y + _yAdjust),
          goal.angularVelocity
              + _thetaController.calculate(_currentPose.getRotation().getRadians(), goal.heading),
          // _currentPose.getRotation()
          _drivetrain.getYaw()
      ));
    } else {
      // Feed to the
      autoAimDriver.setChoreoAimDriverVelocities(
          goal.velocityX
              + _xController.calculate(
                  // _currentPose.getX(), _invertTrajectoryAndNASA ? goal.x : goal.x + _xAdjust
                  _currentPose.getX(),
                  goal.x + _xAdjust
              ),
          goal.velocityY + _yController.calculate(_currentPose.getY(), goal.y + _yAdjust)
      );
      autoAimDriver.execute();
    }

    // Schedule all commands mapped to sample points not yet seen
    for (int i = _lastIndex; i <= ahead; i++) {
      if (!_setpointSeen[i]) {
        // System.out.println("index " + i + " seen");
        _setpointSeen[i] = true;
        var action = _commandMap.get(i);
        if (action != null) {
          action.command.schedule();
          System.out.println(action.name());
          System.out.flush();
        }
      }
    }
    _lastIndex = ahead;
  }

  @Override
  public void end(boolean interrupted) {
    _timer.stop();
    autoAimDriver.end(false);
    if (interrupted) {
      _drivetrain.robotCentricDrive(new ChassisSpeeds());
    } else {
      _drivetrain.robotCentricDrive(_finalState.getChassisSpeeds());
    }
  }

  @Override
  public boolean isFinished() {
    // this is probably off by one
    return _timer.hasElapsed(_trajectory.getTotalTime())
        && (!_stopAtEnd || _drivetrain.atPose(_currentPose, _finalPose));
  }

  public static void setAimingMode(boolean aiming) {
    if (aiming) {
      autoAimDriver.initialize();
    } else {
      autoAimDriver.end(false);
    }
    _aimingMode = aiming;
  }
}
