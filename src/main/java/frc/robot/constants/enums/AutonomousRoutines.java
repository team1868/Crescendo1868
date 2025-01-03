package frc.robot.constants.enums;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.complex.AimAllAtSpeakerCommand;
import frc.robot.constants.enums.AutonomousType.BuilderType;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShotConstants;
import frc.robot.utils.TagalongChoreoFollower;
import frc.robot.utils.TagalongChoreoTrajectory;
import java.util.ArrayList;
import java.util.Map;
import tagalong.commands.base.PivotToCmd;

public enum AutonomousRoutines {
  DEFAULT_AUTO(
      "DEFAULT",
      LedColors.BLOOD_RED,
      Commands.print("DEFAULT AUTO SAYS HI"),
      PrematchOdometryModes.FULL_VISION
  ),
  SOURCE_LEAVE(
      ShuffleboardStatus.SHOW,
      "SOURCE LEAVE",
      LedColors.OFF,
      "SOURCE_LEAVE",
      new AutonomousActions[] {AutonomousActions.SOURCESTART_SHOT_COMMAND},
      new int[] {1},
      Map.ofEntries(),
      PrematchOdometryModes.NO_VISION
  ),
  SOURCE_LEAVE_KNOCKOUT(
      ShuffleboardStatus.SHOW,
      "SOURCE LEAVE KNOCKOUT",
      LedColors.OFF,
      "SOURCE_LEAVE_KNOCKOUT",
      new AutonomousActions[] {AutonomousActions.SOURCESTART_SHOT_COMMAND},
      new int[] {1},
      Map.ofEntries(),
      PrematchOdometryModes.NO_VISION
  ),
  SOURCE_2PIECE(
      ShuffleboardStatus.SHOW,
      "SOURCE 2P",
      LedColors.OFF,
      "SOURCE_2P",
      new AutonomousActions[] {
          AutonomousActions.SOURCESTART_SHOT_COMMAND, AutonomousActions.N3_SHOT_COMMAND},
      new int[] {1, 2},
      Map.ofEntries(
          Map.entry(0, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(1, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND)))
      ),
      PrematchOdometryModes.NO_VISION
  ),
  MID_2PIECE(
      ShuffleboardStatus.SHOW,
      "MID 2P",
      LedColors.OFF,
      "MID_2P",
      new AutonomousActions[] {
          AutonomousActions.MIDSTART_SHOT_COMMAND, AutonomousActions.N2_SHOT_COMMAND},
      new int[] {1, 2},
      Map.ofEntries(
          Map.entry(0, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(1, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND)))
      ),
      PrematchOdometryModes.NO_VISION
  ),
  AMP_2PIECE(
      ShuffleboardStatus.HIDE,
      "AMP SIDE 2P",
      LedColors.OFF,
      "AMP_2P",
      new AutonomousActions[] {
          AutonomousActions.AMPSTART_SHOT_COMMAND, AutonomousActions.N1_SHOT_COMMAND},
      new int[] {1, 2},
      Map.ofEntries(
          Map.entry(0, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(1, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND)))
      ),
      PrematchOdometryModes.NO_VISION
  ),
  // AMP_RACE_3PIECE(
  // ShuffleboardStatus.HIDE,
  // "AMP RACE 3P",
  // LedColors.OFF,
  // "AMP_4R_N451",
  // new AutonomousActions[] {},
  // new int[] {},
  // Map.ofEntries(
  // Map.entry(
  // 0,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.STOP_AUTO_AIMING),
  // Map.entry(100, AutonomousActions.START_SHOOTING),
  // Map.entry(120, AutonomousActions.FORCE_SHOT),
  // Map.entry(130, AutonomousActions.STOP_SHOOTING),
  // Map.entry(190, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(195, AutonomousActions.INTAKE_COMMAND)
  // )
  // ),
  // Map.entry( // intake n4
  // 1,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.INTAKE_COMMAND),
  // Map.entry(15, AutonomousActions.POST_INTAKE_COMMAND),
  // Map.entry(120, AutonomousActions.START_SHOOTING),
  // Map.entry(160, AutonomousActions.FORCE_SHOT),
  // Map.entry(165, AutonomousActions.STOP_SHOOTING),
  // Map.entry(194, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(195, AutonomousActions.INTAKE_COMMAND)
  // )
  // ),
  // Map.entry( // intake n5
  // 2,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.INTAKE_COMMAND),
  // Map.entry(15, AutonomousActions.POST_INTAKE_COMMAND),
  // Map.entry(50, AutonomousActions.START_SHOOTING),
  // Map.entry(80, AutonomousActions.FORCE_SHOT),
  // Map.entry(90, AutonomousActions.STOP_SHOOTING),
  // Map.entry(192, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(195, AutonomousActions.INTAKE_COMMAND)
  // )
  // ),
  // Map.entry( // intake n1
  // 3,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.INTAKE_COMMAND),
  // Map.entry(39, AutonomousActions.START_SHOOTING)
  // )
  // )
  // ),
  // PrematchOdometryModes.NO_VISION,
  // true
  // ),
  // SOURCE_4P_321_1LEG(
  // ShuffleboardStatus.HIDE,
  // "SOURCE 4P 321",
  // LedColors.OFF,
  // "SOURCE_4P_321_1LEG",
  // new AutonomousActions[] {},
  // new int[] {},
  // Map.ofEntries(
  // Map.entry( // shoot pre, intake n3
  // 0,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.STOP_AUTO_AIMING),
  // Map.entry(5, AutonomousActions.START_SHOOTING),
  // Map.entry(10, AutonomousActions.FORCE_SHOT),
  // Map.entry(15, AutonomousActions.STOP_SHOOTING),
  // Map.entry(20, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(30, AutonomousActions.INTAKE_COMMAND)
  // )
  // ),
  // Map.entry( // intake n3, shoot n3
  // 1,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.INTAKE_COMMAND),
  // Map.entry(15, AutonomousActions.POST_INTAKE_COMMAND),
  // Map.entry(25, AutonomousActions.START_SHOOTING),
  // Map.entry(30, AutonomousActions.FORCE_SHOT)
  // )
  // ),
  // Map.entry( // shoot n3, intake n2
  // 2,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.STOP_SHOOTING),
  // Map.entry(5, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(10, AutonomousActions.INTAKE_COMMAND)
  // )
  // ),
  // Map.entry( // intake n2, shoot n2
  // 3,
  // Map.ofEntries(
  // Map.entry(15, AutonomousActions.POST_INTAKE_COMMAND),
  // Map.entry(20, AutonomousActions.START_SHOOTING),
  // Map.entry(30, AutonomousActions.FORCE_SHOT),
  // Map.entry(35, AutonomousActions.STOP_SHOOTING)
  // )
  // ),
  // Map.entry( // intake n1
  // 4,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(13, AutonomousActions.INTAKE_COMMAND),
  // Map.entry(35, AutonomousActions.POST_INTAKE_COMMAND)
  // )
  // )
  // ),
  // PrematchOdometryModes.NO_VISION,
  // true
  // ),
  MID_6P_321( // way 3 stop pt off
      ShuffleboardStatus.SHOW,
      "MID 6P 321",
      LedColors.OFF,
      "MID_6P_321",
      new AutonomousActions[] {
          AutonomousActions.MIDSTART_SHOT_COMMAND, // 0, way1
          AutonomousActions.N3_SHOT_COMMAND, // 1, way3
          AutonomousActions.N2_SHOT_COMMAND, // 2, way4
          AutonomousActions.N1_SHOT_COMMAND, // 3, way6
          // intake n4, 4, way8
          AutonomousActions.N4_SHOT_COMMAND, // 5, way9
          // intake n5, 6, way11
          AutonomousActions.N5_SHOT_COMMAND}, // 7, end
      new int[] {0, 1, 2, 3, 5, 7},
      Map.ofEntries(
          Map.entry(0, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(1, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(2, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(3, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(5, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND)))
      ),
      PrematchOdometryModes.NO_VISION
  ),
  SUNNY_MID_6P_321( // way 3 stop pt off
      ShuffleboardStatus.SHOW,
      "SUNNY MID 6P 321",
      LedColors.OFF,
      "SUNNY_MID_6P_321",
      new AutonomousActions[] {
          AutonomousActions.MIDSTART_SHOT_COMMAND, // 0, way1
          AutonomousActions.N3_SHOT_COMMAND, // 1, way3
          AutonomousActions.N2_SHOT_COMMAND, // 2, way4
          AutonomousActions.N1_SHOT_COMMAND, // 3, way6
          // intake n4, 4, way8
          AutonomousActions.N4_SHOT_COMMAND, // 5, way9
          // intake n5, 6, way11
          AutonomousActions.N5_SHOT_COMMAND}, // 7, end
      new int[] {0, 1, 2, 3, 5, 7},
      Map.ofEntries(
          Map.entry(0, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(1, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(2, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(3, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(5, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND)))
      ),
      PrematchOdometryModes.NO_VISION
  ),
  MID_5P_321( // way 3 stop pt off
      ShuffleboardStatus.SHOW,
      "MID 5P 321",
      LedColors.OFF,
      "MID_5P_321",
      new AutonomousActions[] {
          AutonomousActions.MIDSTART_SHOT_COMMAND, // 0, way1
          AutonomousActions.N3_SHOT_COMMAND, // 1, way3
          AutonomousActions.N2_SHOT_COMMAND, // 2, way4
          AutonomousActions.N1_SHOT_COMMAND, // 3, way6
          // intake n4, 4,
          AutonomousActions.N4_SHOT_COMMAND}, // 5, end
      new int[] {0, 1, 2, 3, 5},
      Map.ofEntries(
          Map.entry(0, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(1, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(2, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(3, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(4, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND)))
      ),
      PrematchOdometryModes.NO_VISION
  ),
  SUNNY_MID_5P_321( // way 3 stop pt off
      ShuffleboardStatus.SHOW,
      "SUNNY MID 5P 321",
      LedColors.OFF,
      "SUNNY_MID_5P_321",
      new AutonomousActions[] {
          AutonomousActions.MIDSTART_SHOT_COMMAND, // 0, way1
          AutonomousActions.N3_SHOT_COMMAND, // 1, way3
          AutonomousActions.N2_SHOT_COMMAND, // 2, way4
          AutonomousActions.N1_SHOT_COMMAND, // 3, way6
          // intake n4, 4,
          AutonomousActions.N4_SHOT_COMMAND}, // 5, end
      new int[] {0, 1, 2, 3, 5},
      Map.ofEntries(
          Map.entry(0, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(1, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(2, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(3, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(4, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND)))
      ),
      PrematchOdometryModes.NO_VISION
  ),
  MID_6P_321_SAFE( // way 3 stop pt off
      ShuffleboardStatus.SHOW,
      "MID 6P 321 SAFE",
      LedColors.OFF,
      "MID_6P_321_SAFE",
      new AutonomousActions[] {
          AutonomousActions.MIDSTART_SHOT_COMMAND, // 0, way1
          AutonomousActions.N3_SHOT_COMMAND, // 1, way3
          AutonomousActions.N2_SHOT_COMMAND, // 2, way4
          AutonomousActions.N1_SHOT_COMMAND, // 3, way6
          // intake n4, 4, way9
          AutonomousActions.N4_SHOT_COMMAND, // 5, way12
          // intake n5, 6, way14
          AutonomousActions.N5_SHOT_COMMAND}, // 7, end
      new int[] {0, 1, 2, 3, 5, 7},
      Map.ofEntries(
          Map.entry(0, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(1, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(2, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(3, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(5, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND)))
      ),
      PrematchOdometryModes.NO_VISION
  ),
  MID_6P_21(
      ShuffleboardStatus.SHOW,
      "MID 6P 21",
      LedColors.OFF,
      "MID_6P_21",
      new AutonomousActions[] {
          AutonomousActions.MIDSTART_SHOT_COMMAND, // 0, way1
          AutonomousActions.N2_SHOT_COMMAND, // 1, way2
          AutonomousActions.N1_SHOT_COMMAND, // 2, way4
          // intake n4, 3, way6
          AutonomousActions.N4_SHOT_COMMAND, // 4, way7
          // intake n5, 5, way9
          AutonomousActions.N5_SHOT_COMMAND, // 6, way11
          // intake n6, 7, way14
          AutonomousActions.N6_SHOT_COMMAND}, // 8, end
      new int[] {0, 1, 2, 4, 6, 8},
      Map.ofEntries(
          Map.entry(0, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(1, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(2, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(3, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(4, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(5, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(6, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(7, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND)))
      ),
      PrematchOdometryModes.NO_VISION
  ),
  SUNNY_MID_6P_21(
      ShuffleboardStatus.SHOW,
      "SUNNY MID 6P 21",
      LedColors.OFF,
      "SUNNY_MID_6P_21",
      new AutonomousActions[] {
          AutonomousActions.MIDSTART_SHOT_COMMAND, // 0, way1
          AutonomousActions.N2_SHOT_COMMAND, // 1, way2
          AutonomousActions.N1_SHOT_COMMAND, // 2, way4
          // intake n4, 3, way6
          AutonomousActions.N4_SHOT_COMMAND, // 4, way7
          // intake n5, 5, way9
          AutonomousActions.N5_SHOT_COMMAND, // 6, way11
          // intake n6, 7, way14
          AutonomousActions.N6_SHOT_COMMAND}, // 8, end
      new int[] {0, 1, 2, 4, 6, 8},
      Map.ofEntries(
          Map.entry(0, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(1, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(2, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(3, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(4, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(5, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(6, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(7, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND)))
      ),
      PrematchOdometryModes.NO_VISION
  ),
  SOURCE_3P_RACE(
      ShuffleboardStatus.HIDE,
      "SOURCE 3P RACE",
      LedColors.OFF,
      "SOURCE_3P_RACE",
      new AutonomousActions[] {
          AutonomousActions.SOURCESTART_SHOT_COMMAND,
          AutonomousActions.N6_SHOT_COMMAND,
          AutonomousActions.N7_SHOT_COMMAND},
      new int[] {1, 3, 6},
      Map.ofEntries(
          Map.entry(0, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(1, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(2, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(3, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(4, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(5, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(6, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND)))
      ),
      PrematchOdometryModes.NO_VISION
  ),
  SOURCE_5P_RACE(
      ShuffleboardStatus.SHOW,
      "SOURCE 5P RACE",
      LedColors.OFF,
      "SOURCE_5P_RACE",
      new AutonomousActions[] {
          // 0, start
          AutonomousActions.SOURCESTART_SHOT_COMMAND, // 1, way2
          // intake n7, 2, way5
          AutonomousActions.SOURCE_FAR_SHOT_COMMAND, // 3, way8
          // intake n6, 4, way11
          AutonomousActions.SOURCE_FAR_SHOT_COMMAND // 5, way14
          // intake n8, 6, way16
          // AutonomousActions.FAR_N8_SHOT_COMMAND //, // 7, way18
          // AutonomousActions.N3_SHOT_COMMAND // 8, end
      },
      new int[] {1, 3, 5},
      Map.ofEntries(
          Map.entry(0, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(1, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(2, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(3, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(4, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(5, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND)))
      ),
      PrematchOdometryModes.NO_VISION
  ),
  SUNNY_SOURCE_5P_RACE(
      ShuffleboardStatus.SHOW,
      "SUNNY SOURCE 5P RACE",
      LedColors.OFF,
      "SUNNY_SOURCE_5P_RACE",
      new AutonomousActions[] {
          // 0, start
          AutonomousActions.SOURCESTART_SHOT_COMMAND, // 1, way2
          // intake n7, 2, way5
          AutonomousActions.SOURCE_FAR_SHOT_COMMAND, // 3, way8
          // intake n6, 4, way11
          AutonomousActions.SOURCE_FAR_SHOT_COMMAND // 5, way14
          // intake n8, 6, way16
          // AutonomousActions.FAR_N8_SHOT_COMMAND //, // 7, way18
          // AutonomousActions.N3_SHOT_COMMAND // 8, end
      },
      new int[] {1, 3, 5},
      Map.ofEntries(
          Map.entry(0, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(1, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(2, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(3, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(4, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(5, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND)))
      ),
      PrematchOdometryModes.NO_VISION
  ),
  AMP_4P_RACE(
      ShuffleboardStatus.SHOW,
      "AMP 4P RACE",
      LedColors.OFF,
      "AMP_4P_RACE",
      new AutonomousActions[] {
          // 0, start
          AutonomousActions.AMP_FAR_SHOT_COMMAND, // 1, way4
          // intake n5, 2, way6
          AutonomousActions.AMP_FAR_SHOT_COMMAND, // 3, way7
          // intake n6, 4, way9
          AutonomousActions.AMP_FAR_SHOT_COMMAND, // 5, way11
          // intake n4, 6, way12
          AutonomousActions.AMP_FAR_SHOT_COMMAND // 7, end
      },
      new int[] {1, 3, 5, 7},
      Map.ofEntries(
          Map.entry(0, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(1, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(2, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(3, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(4, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(5, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND)))
      ),
      PrematchOdometryModes.NO_VISION
  ),
  SUNNY_AMP_4P_RACE(
      ShuffleboardStatus.SHOW,
      "SUNNY AMP 4P RACE",
      LedColors.OFF,
      "SUNNY_AMP_4P_RACE",
      new AutonomousActions[] {
          // 0, start
          AutonomousActions.AMP_FAR_SHOT_COMMAND, // 1, way4
          // intake n5, 2, way6
          AutonomousActions.AMP_FAR_SHOT_COMMAND, // 3, way7
          // intake n6, 4, way9
          AutonomousActions.AMP_FAR_SHOT_COMMAND, // 5, way11
          // intake n4, 6, way12
          AutonomousActions.AMP_FAR_SHOT_COMMAND // 7, end
      },
      new int[] {1, 3, 5, 7},
      Map.ofEntries(
          Map.entry(0, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(1, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(2, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(3, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(4, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND))),
          Map.entry(5, Map.ofEntries(Map.entry(1, AutonomousActions.INTAKE_COMMAND)))
      ),
      PrematchOdometryModes.NO_VISION
  ),
  // AMP_5P_N4567(
  // ShuffleboardStatus.HIDE,
  // "Amp 5 Race 4567",
  // LedColors.OFF,
  // "AMP_5P_N4567",
  // new AutonomousActions[] {},
  // new int[] {},
  // Map.ofEntries(
  // Map.entry( // shoot pre, intake n4
  // 0,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.STOP_AUTO_AIMING),
  // Map.entry(40, AutonomousActions.START_SHOOTING),
  // Map.entry(80, AutonomousActions.FORCE_SHOT),
  // Map.entry(85, AutonomousActions.STOP_SHOOTING),
  // Map.entry(100, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(112, AutonomousActions.INTAKE_COMMAND)
  // )
  // ),
  // Map.entry( // intake n4, shoot n4, intake n5
  // 1,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.INTAKE_COMMAND),
  // Map.entry(5, AutonomousActions.POST_INTAKE_COMMAND),
  // Map.entry(25, AutonomousActions.START_SHOOTING),
  // Map.entry(40, AutonomousActions.FORCE_SHOT),
  // Map.entry(45, AutonomousActions.STOP_SHOOTING),
  // Map.entry(100, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(112, AutonomousActions.INTAKE_COMMAND)
  // )
  // ),
  // Map.entry( // intake n5, shoot n5, intake n6
  // 2,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.INTAKE_COMMAND),
  // Map.entry(5, AutonomousActions.POST_INTAKE_COMMAND),
  // Map.entry(50, AutonomousActions.START_SHOOTING),
  // Map.entry(80, AutonomousActions.FORCE_SHOT),
  // Map.entry(85, AutonomousActions.STOP_SHOOTING),
  // Map.entry(160, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(190, AutonomousActions.INTAKE_COMMAND)
  // )
  // ),
  // Map.entry( // intake n6, shoot n6, intake n7
  // 3,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.INTAKE_COMMAND),
  // Map.entry(5, AutonomousActions.POST_INTAKE_COMMAND),
  // Map.entry(60, AutonomousActions.START_SHOOTING),
  // Map.entry(80, AutonomousActions.FORCE_SHOT),
  // Map.entry(85, AutonomousActions.STOP_SHOOTING),
  // Map.entry(120, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(150, AutonomousActions.INTAKE_COMMAND)
  // )
  // ),
  // Map.entry( // intake n7, shoot n7
  // 4,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.INTAKE_COMMAND),
  // Map.entry(5, AutonomousActions.POST_INTAKE_COMMAND)
  // )
  // )
  // ),
  // PrematchOdometryModes.NO_VISION,
  // true
  // ),
  // AMP_5P_N4561(
  // ShuffleboardStatus.HIDE,
  // "Amp 5 Race 4561",
  // LedColors.OFF,
  // "AMP_5P_N4561",
  // new AutonomousActions[] {},
  // new int[] {},
  // Map.ofEntries(
  // Map.entry( // shoot pre, intake n4
  // 0,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.STOP_AUTO_AIMING),
  // Map.entry(75, AutonomousActions.START_SHOOTING),
  // Map.entry(85, AutonomousActions.FORCE_SHOT),
  // Map.entry(90, AutonomousActions.STOP_SHOOTING),
  // Map.entry(93, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(105, AutonomousActions.INTAKE_COMMAND)
  // )
  // ),
  // Map.entry( // intake n4, shoot n4, intake n5
  // 1,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.INTAKE_COMMAND),
  // Map.entry(15, AutonomousActions.POST_INTAKE_COMMAND),
  // Map.entry(18, AutonomousActions.START_SHOOTING),
  // Map.entry(30, AutonomousActions.FORCE_SHOT),
  // Map.entry(43, AutonomousActions.STOP_SHOOTING),
  // Map.entry(45, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(85, AutonomousActions.INTAKE_COMMAND)
  // )
  // ),
  // Map.entry( // intake n5, shoot n5, intake n6
  // 2,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.INTAKE_COMMAND),
  // Map.entry(10, AutonomousActions.POST_INTAKE_COMMAND),
  // Map.entry(15, AutonomousActions.START_SHOOTING),
  // Map.entry(70, AutonomousActions.FORCE_SHOT),
  // Map.entry(85, AutonomousActions.STOP_SHOOTING),
  // Map.entry(87, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(180, AutonomousActions.INTAKE_COMMAND)
  // )
  // ),
  // Map.entry( // intake n6, shoot n6, intake n1
  // 3,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.INTAKE_COMMAND),
  // Map.entry(15, AutonomousActions.POST_INTAKE_COMMAND),
  // Map.entry(25, AutonomousActions.START_SHOOTING),
  // Map.entry(75, AutonomousActions.FORCE_SHOT),
  // Map.entry(85, AutonomousActions.STOP_SHOOTING),
  // Map.entry(110, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(150, AutonomousActions.INTAKE_COMMAND)
  // )
  // )
  // ),
  // PrematchOdometryModes.NO_VISION,
  // true
  // ),
  // SOURCE_4R_N673(
  // ShuffleboardStatus.HIDE,
  // "SOURCE 4 Race (673)",
  // LedColors.OFF,
  // "SOURCE_4R_N673",
  // new AutonomousActions[] {AutonomousActions.WAIT_COMMAND},
  // new int[] {0},
  // Map.ofEntries( // TODO: tune
  // Map.entry( // shoot pre, intake n6
  // 0,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.STOP_AUTO_AIMING),
  // Map.entry(10, AutonomousActions.START_SHOOTING),
  // Map.entry(30, AutonomousActions.FORCE_SHOT),
  // Map.entry(40, AutonomousActions.STOP_SHOOTING),
  // Map.entry(150, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(200, AutonomousActions.INTAKE_COMMAND)
  // )
  // ),
  // Map.entry( // intake n6, shoot n6, intake n7
  // 1,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.INTAKE_COMMAND),
  // Map.entry(20, AutonomousActions.POST_INTAKE_COMMAND),
  // Map.entry(30, AutonomousActions.START_SHOOTING),
  // Map.entry(80, AutonomousActions.FORCE_SHOT),
  // Map.entry(90, AutonomousActions.STOP_SHOOTING),
  // Map.entry(100, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(145, AutonomousActions.INTAKE_COMMAND)
  // )
  // ),
  // Map.entry( // intake n7, shoot n7, intake n3
  // 2,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.INTAKE_COMMAND),
  // Map.entry(20, AutonomousActions.POST_INTAKE_COMMAND),
  // Map.entry(80, AutonomousActions.START_SHOOTING),
  // Map.entry(100, AutonomousActions.FORCE_SHOT),
  // Map.entry(110, AutonomousActions.STOP_SHOOTING),
  // Map.entry(120, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(145, AutonomousActions.INTAKE_COMMAND),
  // Map.entry(159, AutonomousActions.POST_INTAKE_COMMAND) // might be too soon
  // // post
  // )
  // )
  // ),
  // PrematchOdometryModes.NO_VISION,
  // true
  // ),
  // SOURCE_4R_N678(
  // ShuffleboardStatus.HIDE,
  // "SOURCE SIDE 4 Race (678)",
  // LedColors.OFF,
  // "SOURCE_4R_N678",
  // new AutonomousActions[] {AutonomousActions.WAIT_COMMAND},
  // new int[] {0},
  // Map.ofEntries( // TODO: tune
  // Map.entry( // shoot pre, intake n6
  // 0,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.STOP_AUTO_AIMING),
  // Map.entry(10, AutonomousActions.START_SHOOTING),
  // Map.entry(30, AutonomousActions.FORCE_SHOT),
  // Map.entry(40, AutonomousActions.STOP_SHOOTING),
  // Map.entry(150, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(200, AutonomousActions.INTAKE_COMMAND)
  // )
  // ),
  // Map.entry( // intake n6, shoot n6, intake n7
  // 1,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.INTAKE_COMMAND),
  // Map.entry(20, AutonomousActions.POST_INTAKE_COMMAND),
  // Map.entry(30, AutonomousActions.START_SHOOTING),
  // Map.entry(80, AutonomousActions.FORCE_SHOT),
  // Map.entry(90, AutonomousActions.STOP_SHOOTING),
  // Map.entry(100, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(145, AutonomousActions.INTAKE_COMMAND)
  // )
  // ),
  // Map.entry( // intake n7, shoot n7, intake n8
  // 2,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.INTAKE_COMMAND),
  // Map.entry(20, AutonomousActions.POST_INTAKE_COMMAND),
  // Map.entry(30, AutonomousActions.START_SHOOTING),
  // Map.entry(80, AutonomousActions.FORCE_SHOT),
  // Map.entry(120, AutonomousActions.STOP_SHOOTING),
  // Map.entry(130, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(145, AutonomousActions.INTAKE_COMMAND)
  // )
  // ),
  // Map.entry( // intake n8, shoot n8
  // 3,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.INTAKE_COMMAND),
  // Map.entry(20, AutonomousActions.POST_INTAKE_COMMAND),
  // Map.entry(60, AutonomousActions.START_SHOOTING)
  // )
  // )
  // ),
  // PrematchOdometryModes.NO_VISION,
  // true
  // ),
  // SCORE_PRELOAD(
  // ShuffleboardStatus.SHOW,
  // "SCORE PRELOAD",
  // LedColors.OFF,
  // AutonomousActions.START_SHOOTING.command.withTimeout(3.0),
  // PrematchOdometryModes.FULL_VISION
  // ),
  // N6783(
  // ShuffleboardStatus.HIDE,
  // "4P N678",
  // LedColors.OFF,
  // "SOURCE_N6783",
  // new AutonomousActions[] {AutonomousActions.WAIT_COMMAND},
  // new int[] {0},
  // Map.ofEntries(
  // Map.entry(
  // 0,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.START_SHOOTING),
  // Map.entry(10, AutonomousActions.FORCE_SHOT),
  // Map.entry(12, AutonomousActions.STOP_SHOOTING),
  // Map.entry(16, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(17, AutonomousActions.INTAKE_COMMAND)
  // )
  // ),
  // Map.entry(
  // 1,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.INTAKE_COMMAND),
  // Map.entry(2, AutonomousActions.POST_INTAKE_COMMAND)
  // )
  // ),
  // Map.entry(
  // 2,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(20, AutonomousActions.INTAKE_COMMAND)
  // )
  // ),
  // Map.entry(
  // 3,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.START_SHOOTING),
  // Map.entry(20, AutonomousActions.FORCE_SHOT),
  // Map.entry(33, AutonomousActions.STOP_SHOOTING),
  // Map.entry(34, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(35, AutonomousActions.INTAKE_COMMAND)

  // )
  // ),
  // Map.entry(
  // 4,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.START_SHOOTING),
  // Map.entry(20, AutonomousActions.FORCE_SHOT),
  // Map.entry(33, AutonomousActions.STOP_SHOOTING),
  // Map.entry(34, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(35, AutonomousActions.INTAKE_COMMAND)

  // )
  // ),
  // Map.entry(
  // 5,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.START_SHOOTING),
  // Map.entry(20, AutonomousActions.FORCE_SHOT),
  // Map.entry(33, AutonomousActions.STOP_SHOOTING),
  // Map.entry(34, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(35, AutonomousActions.INTAKE_COMMAND)

  // )
  // ),
  // Map.entry(
  // 6,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.START_SHOOTING),
  // Map.entry(20, AutonomousActions.FORCE_SHOT),
  // Map.entry(33, AutonomousActions.STOP_SHOOTING),
  // Map.entry(34, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(35, AutonomousActions.INTAKE_COMMAND)

  // )
  // ),
  // Map.entry(
  // 7,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.START_SHOOTING),
  // Map.entry(20, AutonomousActions.FORCE_SHOT),
  // Map.entry(33, AutonomousActions.STOP_SHOOTING),
  // Map.entry(34, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(35, AutonomousActions.INTAKE_COMMAND)

  // )
  // ),
  // Map.entry(
  // 8,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.START_SHOOTING),
  // Map.entry(20, AutonomousActions.FORCE_SHOT),
  // Map.entry(33, AutonomousActions.STOP_SHOOTING),
  // Map.entry(34, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(35, AutonomousActions.INTAKE_COMMAND)

  // )
  // ),
  // Map.entry(
  // 9,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.START_SHOOTING),
  // Map.entry(20, AutonomousActions.FORCE_SHOT),
  // Map.entry(33, AutonomousActions.STOP_SHOOTING),
  // Map.entry(34, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(35, AutonomousActions.INTAKE_COMMAND)

  // )
  // ),
  // Map.entry(
  // 10,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.START_SHOOTING),
  // Map.entry(20, AutonomousActions.FORCE_SHOT),
  // Map.entry(33, AutonomousActions.STOP_SHOOTING),
  // Map.entry(34, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(35, AutonomousActions.INTAKE_COMMAND)

  // )
  // ),
  // Map.entry(
  // 11,
  // Map.ofEntries(
  // Map.entry(1, AutonomousActions.START_SHOOTING),
  // Map.entry(20, AutonomousActions.FORCE_SHOT),
  // Map.entry(33, AutonomousActions.STOP_SHOOTING),
  // Map.entry(34, AutonomousActions.PREP_TO_INTAKE_COMMAND),
  // Map.entry(35, AutonomousActions.INTAKE_COMMAND)

  // )
  // )
  // ),
  // PrematchOdometryModes.NO_VISION
  // ),
  N3_5P_3214(
      ShuffleboardStatus.HIDE,
      "N3 5P 3214",
      LedColors.OFF,
      "N3A_5P_3214NOSP",
      PrematchOdometryModes.NO_VISION
  );

  AutonomousRoutines(
      ShuffleboardStatus show,
      String shuffleboardName,
      LedColors prepColor,
      String trajectoryName,
      AutonomousActions[] intermediateCommands,
      int[] interruptionPoints,
      Map<Integer, Map<Integer, AutonomousActions>> mappedActions,
      PrematchOdometryModes odomSetup
  ) {
    showInDashboard = show;
    this.shuffleboardName = shuffleboardName;
    this.trajectoryName = trajectoryName;
    this.intermediateCommands = intermediateCommands;
    this.interruptionPoints = interruptionPoints;
    this.prepColor = prepColor;
    category = AutonomousType.SPLICED_AND_MAPPED_TAGALONG;
    this.odomSetup = odomSetup;
    this.mappedActions = mappedActions;
  }

  AutonomousRoutines(
      ShuffleboardStatus show,
      String shuffleboardName,
      LedColors prepColor,
      String trajectoryName,
      AutonomousActions[] intermediateCommands,
      int[] interruptionPoints,
      Map<Integer, Map<Integer, AutonomousActions>> mappedActions,
      PrematchOdometryModes odomSetup,
      boolean forceLastShot
  ) {
    showInDashboard = show;
    this.shuffleboardName = shuffleboardName;
    this.trajectoryName = trajectoryName;
    this.intermediateCommands = intermediateCommands;
    this.interruptionPoints = interruptionPoints;
    this.prepColor = prepColor;
    category = AutonomousType.SPLICED_AND_MAPPED_TAGALONG_WITH_FORCED_SHOT;
    this.odomSetup = odomSetup;
    this.mappedActions = mappedActions;
  }

  // ShuffleboardStatus show,
  // String shuffleboardName,
  // LedColors prepColor,
  // String trajectoryName,
  // AutonomousActions[] intermediateCommands,
  // int[] interruptionPoints,
  // AutonomouseAction[] mappedCommands
  // PrematchOdometryModes odomSetup
  public static final Controlboard _controlboard = Controlboard.get();
  public final ShuffleboardStatus showInDashboard;
  public final String shuffleboardName;
  public final String trajectoryName;
  public final LedColors prepColor;
  public final AutonomousType category;
  public final PrematchOdometryModes odomSetup;

  public AutonomousActions[] intermediateCommands;
  public int[] interruptionPoints;

  public ArrayList<ChoreoTrajectory> trajectory = new ArrayList<ChoreoTrajectory>();
  public ArrayList<TagalongChoreoTrajectory> tagalongTrajectory =
      new ArrayList<TagalongChoreoTrajectory>();

  public Command[] commands;
  public Command builtCommand;
  Map<Integer, Map<Integer, AutonomousActions>> mappedActions;

  AutonomousRoutines(
      String shuffleboardName,
      LedColors prepColor,
      Command simpleCommand,
      PrematchOdometryModes odomSetup
  ) {
    this(ShuffleboardStatus.SHOW, shuffleboardName, prepColor, simpleCommand, odomSetup);
  }

  AutonomousRoutines(
      ShuffleboardStatus show,
      String shuffleboardName,
      LedColors prepColor,
      Command simpleCommand,
      PrematchOdometryModes odomSetup
  ) {
    showInDashboard = show;
    this.shuffleboardName = shuffleboardName;
    this.prepColor = prepColor;
    commands = new Command[] {simpleCommand};
    category = AutonomousType.BASIC_COMMAND;
    this.odomSetup = odomSetup;

    trajectoryName = null;
    intermediateCommands = null;
  }

  AutonomousRoutines(
      String shuffleboardName,
      LedColors prepColor,
      String trajectoryName,
      PrematchOdometryModes odomSetup
  ) {
    this(ShuffleboardStatus.SHOW, shuffleboardName, prepColor, trajectoryName, odomSetup);
  }

  AutonomousRoutines(
      ShuffleboardStatus show,
      String shuffleboardName,
      LedColors prepColor,
      String trajectoryName,
      PrematchOdometryModes odomSetup
  ) {
    showInDashboard = show;
    this.shuffleboardName = shuffleboardName;
    this.trajectoryName = trajectoryName;
    this.prepColor = prepColor;
    category = AutonomousType.BASIC_CHOREO;
    this.odomSetup = odomSetup;

    intermediateCommands = null;
  }

  AutonomousRoutines(
      String shuffleboardName,
      LedColors prepColor,
      String trajectoryName,
      AutonomousActions[] intermediateCommands,
      int[] interruptionPoints,
      PrematchOdometryModes odomSetup
  ) {
    this(
        ShuffleboardStatus.SHOW,
        shuffleboardName,
        prepColor,
        trajectoryName,
        intermediateCommands,
        interruptionPoints,
        odomSetup
    );
  }

  AutonomousRoutines(
      ShuffleboardStatus show,
      String shuffleboardName,
      LedColors prepColor,
      String trajectoryName,
      AutonomousActions[] intermediateCommands,
      int[] interruptionPoints,
      PrematchOdometryModes odomSetup
  ) {
    showInDashboard = show;
    this.shuffleboardName = shuffleboardName;
    this.trajectoryName = trajectoryName;
    this.intermediateCommands = intermediateCommands;
    this.interruptionPoints = interruptionPoints;
    this.prepColor = prepColor;
    category = AutonomousType.SPLICED_CHOREO;
    this.odomSetup = odomSetup;
  }

  public void build(Drivetrain drivetrain, Shooter shooter, Intake intake, Leds leds) {
    if (showInDashboard.show) {
      switch (category) {
        case BASIC_COMMAND:
          basicCommandBuilder(drivetrain);
          break;
        case BASIC_CHOREO:
        case BASIC_TAGALONG:
          basicPathBuilder(drivetrain);
          break;
        case SPLICED_CHOREO:
        case SPLICE_TAGALONG:
          buildSplicedAuto(drivetrain);
          break;
        case SPLICED_AND_MAPPED_TAGALONG:
          buildMappedAuto(drivetrain, shooter, intake, leds);
          break;
        case SPLICED_AND_MAPPED_TAGALONG_WITH_FORCED_SHOT:
          buildAndThenMappedAuto(drivetrain, shooter, intake);
          builtCommand = builtCommand.andThen(Commands.deadline(
              Commands
                  .parallel(
                      Commands.waitSeconds(1.0),
                      new InstantCommand(() -> AimAllAtSpeakerCommand.startShooting(true))
                  )
                  .andThen(
                      new InstantCommand(() -> AimAllAtSpeakerCommand.forceShot()),
                      Commands.waitSeconds(3.0),
                      new InstantCommand(() -> AimAllAtSpeakerCommand.stopShooting())
                  ),
              new AimAllAtSpeakerCommand(drivetrain, shooter, intake)
          ));
          break;

        default:
          System.err.println("Unknown autonomous type");
      }
    }
  }

  public void basicCommandBuilder(Drivetrain drivetrain) {
    builtCommand = buildNetCommandWithOdom(drivetrain);
  }

  private Pose2d spliceVisionPose(Pose2d vision, Pose2d start) {
    switch (odomSetup) {
      case FULL_VISION:
        return vision;
      case VISION_NO_X:
        return new Pose2d(start.getX(), vision.getY(), vision.getRotation());
      case VISION_NO_Y:
        return new Pose2d(vision.getX(), start.getY(), vision.getRotation());
      case VISION_NO_ANGLE:
        return new Pose2d(vision.getX(), vision.getY(), start.getRotation());
      case VISION_NO_XY:
        return new Pose2d(start.getX(), start.getY(), vision.getRotation());
      case VISION_NO_XANGLE:
        return new Pose2d(start.getX(), vision.getY(), start.getRotation());
      case VISION_NO_YANGLE:
        return new Pose2d(vision.getX(), start.getY(), start.getRotation());
      case NO_VISION:
      default:
        return start;
    }
  }

  private Command buildNetCommandWithOdom(Drivetrain drivetrain) {
    switch (category._type) {
      case TAGALONG:
      case CHOREO:
        return new InstantCommand(
                   () -> {
                     var startPose = spliceVisionPose(
                         drivetrain.getPose(),
                         Controlboard.isRedAlliance() ? trajectory.get(0).flipped().getInitialPose()
                                                      : trajectory.get(0).getInitialPose()
                     );
                     drivetrain.setPose(startPose, startPose.getRotation());
                   }
        ).andThen(commands);
      case MAPPED_TAGALONG:
        return new InstantCommand(
                   () -> {
                     var startPose = spliceVisionPose(
                         drivetrain.getPose(),
                         Controlboard.isRedAlliance()
                             ? tagalongTrajectory.get(0).flipped().getInitialPose()
                             : tagalongTrajectory.get(0).getInitialPose()
                     );
                     drivetrain.setPose(startPose, startPose.getRotation());
                   }
        ).andThen(commands);
      case NOT_BUILDABLE:
      default:
        return new InstantCommand(
                   () -> {
                     var startPose = spliceVisionPose(
                         drivetrain.getPose(),
                         Controlboard.isRedAlliance()
                             ? new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0))
                             : new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180.0))
                     );
                     drivetrain.setPose(startPose, startPose.getRotation());
                   }
        ).andThen(commands);
    }
  }

  public Command[] getChoreoCommands(Drivetrain drivetrain) {
    Command[] choreoCommands = new Command[trajectory.size()];
    Controlboard controlboard = Controlboard.get();
    // Create a swerve commands for the robot to follow the trajectory
    for (int i = 0; i < trajectory.size(); i++) {
      choreoCommands[i] = Choreo.choreoSwerveCommand(
          trajectory.get(i), // Choreo trajectory from above
          drivetrain::getPose, // A function that returns the current field-relative pose of
                               // the robot: your wheel or vision odometry
          // TODO move and make more configurable
          new PIDController(5.0, 0.0, 0.0), // PIDController for field-relative X translation
                                            // (input: X error in meters, output: m/s).
          new PIDController(5.0, 0.0, 0.0), // PIDController for field-relative Y translation
                                            // (input: Y error in meters, output: m/s).
          new PIDController(2.0, 0.0, 0.0), // PID constants to correct for rotation error
          drivetrain::robotCentricDrive, // A function that consumes the target
                                         // robot-relative chassis speeds and commandss them
                                         // to the robot
          Controlboard::isRedAlliance, // Whether or not to mirror the path based on
                                       // alliance (this assumes the path is created for the
                                       // blue alliance)
          drivetrain // The subsystem(s) to require, typically your drive subsystem (this)
                     // only
      );
    }
    return choreoCommands;
  }

  public Command[] getTagalongCommands(Drivetrain drivetrain) {
    Command[] tagalongCommands = new Command[trajectory.size()];
    Controlboard controlboard = Controlboard.get();
    // Create a swerve commands for the robot to follow the trajectory
    for (int i = 0; i < tagalongTrajectory.size(); i++) {
      tagalongCommands[i] = TagalongChoreo.choreoSwerveCommand(
          tagalongTrajectory.get(i), // Choreo trajectory from above
          drivetrain::getPose, // A function that returns the current field-relative pose of
                               // the robot: your wheel or vision odometry
          // TODO move and make more configurable
          new PIDController(5.0, 0.0, 0.0), // PIDController for field-relative X translation
                                            // (input: X error in meters, output: m/s).
          new PIDController(5.0, 0.0, 0.0), // PIDController for field-relative Y translation
                                            // (input: Y error in meters, output: m/s).
          new PIDController(2.0, 0.0, 0.0), // PID constants to correct for rotation error
          drivetrain::robotCentricDrive, // A function that consumes the target
                                         // robot-relative chassis speeds and commandss them
                                         // to the robot
          Controlboard::isRedAlliance, // Whether or not to mirror the path based on
                                       // alliance
          // (this assumes the path is created for the blue alliance)
          drivetrain // The subsystem(s) to require, typically your drive subsystem (this)
                     // only
      );
    }
    return tagalongCommands;
  }

  public void basicPathBuilder(Drivetrain drivetrain) {
    if (category._type == BuilderType.TAGALONG) {
      tagalongTrajectory = TagalongChoreo.getTrajectoryGroup(trajectoryName);
      commands = getTagalongCommands(drivetrain);
    } else {
      trajectory = Choreo.getTrajectoryGroup(trajectoryName);
      commands = getChoreoCommands(drivetrain);
    }

    builtCommand = buildNetCommandWithOdom(drivetrain);
  }

  private void spliceCommands(Command[] pathCommands) {
    // System.out.println("event map = " + AutonomousActions.getEventMap())
    // System.out.println("event map = " + AutonomousActions.getEventMap())
    Map<AutonomousActions, Command> eventMap = AutonomousActions.getEventMap();
    commands = new Command[pathCommands.length + intermediateCommands.length];
    int splicedIndex = 0;
    int netIndex = 0;

    for (int i = 0; i < pathCommands.length; i++) {
      for (int j = splicedIndex; j < intermediateCommands.length; j++) {
        if (netIndex == interruptionPoints[j]) {
          commands[netIndex] = eventMap.get(intermediateCommands[j]);
          splicedIndex++;
          netIndex++;
        }
      }
      commands[netIndex] = pathCommands[i];
      netIndex++;
    }
  }

  public void buildMappedAuto(Drivetrain drivetrain, Shooter shooter, Intake intake, Leds leds) {
    if (category._type == BuilderType.MAPPED_TAGALONG) {
      if (intermediateCommands == null) {
        tagalongTrajectory = TagalongChoreo.getTrajectoryGroup(trajectoryName);
        commands = new Command[tagalongTrajectory.size()];

        for (int i = 0; i < tagalongTrajectory.size(); i++) {
          var current = tagalongTrajectory.get(i);
          commands[i] = new TagalongChoreoFollower(
              drivetrain, shooter, intake, current, mappedActions.get(i)
          );
        }

        builtCommand = buildNetCommandWithOdom(drivetrain)
                           .andThen(new AimAllAtSpeakerCommand(drivetrain, shooter, intake));
      } else {
        tagalongTrajectory = TagalongChoreo.getTrajectoryGroup(trajectoryName);
        commands = new Command[tagalongTrajectory.size() + intermediateCommands.length];
        int splicedIndex = 0;
        int netIndex = 0;

        for (int i = 0; i < tagalongTrajectory.size(); i++) {
          for (int j = splicedIndex; j < intermediateCommands.length; j++) {
            if (i == interruptionPoints[j]) {
              var intermediate = intermediateCommands[j];
              commands[netIndex] = intermediate.waitUntilUnderlyingFinished().beforeStarting(
                  () -> intermediate.command.schedule()
              );
              System.out.println("added: " + commands[netIndex].getName());
              splicedIndex++;
              netIndex++;
            }
          }

          var current = tagalongTrajectory.get(i);
          commands[netIndex] = new TagalongChoreoFollower(
              drivetrain, shooter, intake, current, mappedActions.get(i)
          );
          netIndex++;
          System.out.println("added: " + i);
        }
        // place the rest of the commands
        for (int j = splicedIndex; j < intermediateCommands.length; j++) {
          var intermediate = intermediateCommands[j];
          commands[netIndex] = intermediate.waitUntilUnderlyingFinished().beforeStarting(
              () -> intermediate.command.schedule()
          );
          System.out.println("added: " + commands[netIndex].getName());
          splicedIndex++;
          netIndex++;
        }

        builtCommand = buildNetCommandWithOdom(drivetrain)
                           .andThen(new AimAllAtSpeakerCommand(drivetrain, shooter, intake));
      }
    } else {
      builtCommand = Commands.none();
    }
  }

  public void buildAndThenMappedAuto(Drivetrain drivetrain, Shooter shooter, Intake intake) {
    if (category._type == BuilderType.MAPPED_TAGALONG) {
      tagalongTrajectory = TagalongChoreo.getTrajectoryGroup(trajectoryName);

      commands = new Command[tagalongTrajectory.size()];
      for (int i = 0; i < tagalongTrajectory.size(); i++) {
        var current = tagalongTrajectory.get(i);
        commands[i] =
            new TagalongChoreoFollower(drivetrain, shooter, intake, current, mappedActions.get(i));
      }
    }

    builtCommand = buildNetCommandWithOdom(drivetrain);

    System.out.print("DONE BUILDinG");
  }

  public void buildSplicedAuto(Drivetrain drivetrain) {
    if (category._type == BuilderType.TAGALONG) {
      tagalongTrajectory = TagalongChoreo.getTrajectoryGroup(trajectoryName);
      spliceCommands(getTagalongCommands(drivetrain));
    } else {
      trajectory = Choreo.getTrajectoryGroup(trajectoryName);
      spliceCommands(getChoreoCommands(drivetrain));
    }

    builtCommand = buildNetCommandWithOdom(drivetrain);
  }
}
