package frc.robot.utils;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;
import java.util.ArrayList;

/** Utilities to load and follow ChoreoTrajectories */
public class TagalongMultiChoreoFollower extends Command {
  public static final Controlboard _controlboard = Controlboard.get();
  private final Drivetrain _drivetrain;
  private final ArrayList<ChoreoTrajectory> _trajectory;
  private final PIDController _xController;
  private final PIDController _yController;
  private final PIDController _thetaController;
  private final Timer _timer = new Timer();
  private boolean _invertTrajectory;

  private final ChoreoTrajectoryState[] _finalState;
  private final boolean _stopAtEnd;
  private Pose2d _currentPose;
  private Pose2d _finalPose;

  private final int _lastIndex;
  private int _currentIndex;
  private final double[] _endTimes;

  // Assumes you follow the full array list
  public TagalongMultiChoreoFollower(
      Drivetrain drivetrain, ArrayList<ChoreoTrajectory> trajectory
  ) {
    _drivetrain = drivetrain;
    _trajectory = trajectory;

    _lastIndex = _trajectory.size() - 1;
    _endTimes = new double[_trajectory.size() + 1];
    _finalState = new ChoreoTrajectoryState[_trajectory.size()];
    _endTimes[0] = 0;
    for (int i = 0; i < _trajectory.size(); i++) {
      _endTimes[i + 1] = _endTimes[i] + _trajectory.get(i).getTotalTime();
      _finalState[i] = _trajectory.get(i).getFinalState();
    }

    // TODO: change where this comes from
    _xController = new PIDController(5.0, 0.0, 0.0);
    _yController = new PIDController(5.0, 0.0, 0.0);
    _thetaController = new PIDController(5.0, 0.0, 0.0);
    _thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Precompute final values
    _stopAtEnd = _finalState[_trajectory.size() - 1].angularVelocity == 0.0
        && _finalState[_trajectory.size() - 1].velocityX == 0.0
        && _finalState[_trajectory.size() - 1].velocityY == 0.0;
    _finalPose = _finalState[_trajectory.size() - 1].getPose();
  }

  @Override
  public void initialize() {
    _currentIndex = 0;
    _invertTrajectory = Controlboard.isRedAlliance();
    _timer.reset();
  }

  @Override
  public void execute() {
    double currentTime = _timer.get();
    _currentPose = _drivetrain.getPose();
    if (_currentIndex > _trajectory.size()) {
      return;
    }

    if (_endTimes[_currentIndex + 1] > currentTime) {
      _currentIndex += 1;
    }

    ChoreoTrajectoryState goal =
        _trajectory.get(_currentIndex).sample(currentTime, _invertTrajectory);

    _drivetrain.robotCentricDrive(ChassisSpeeds.fromFieldRelativeSpeeds(
        goal.velocityX + _xController.calculate(_currentPose.getX(), goal.x),
        goal.velocityY + _yController.calculate(_currentPose.getY(), goal.y),
        goal.angularVelocity
            + _thetaController.calculate(_currentPose.getRotation().getRadians(), goal.heading),
        _currentPose.getRotation()
    ));
  }

  @Override
  public void end(boolean interrupted) {
    _timer.stop();
    if (interrupted) {
      _drivetrain.robotCentricDrive(new ChassisSpeeds());
    } else {
      _drivetrain.robotCentricDrive(_finalState[_trajectory.size() - 1].getChassisSpeeds());
    }
  }

  @Override
  public boolean isFinished() {
    // this is probably off by one
    return _timer.hasElapsed(_endTimes[_trajectory.size()])
        && (!_stopAtEnd || _drivetrain.atPose(_currentPose, _finalPose));
  }
}
