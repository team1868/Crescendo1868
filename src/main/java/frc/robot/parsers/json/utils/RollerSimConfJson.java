package frc.robot.parsers.json.utils;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.ArrayList;

public class RollerSimConfJson extends TagalongBaseJson {
  public double MOI;
  public int numMotors;
  public int numLigaments;
  public FeedForwardConstantsJson feedforward;
  public MotorConfigJson simMotor;
  private Slot0Configs slot0 = null;
  private Slot1Configs slot1 = null;
  private Slot2Configs slot2 = null;

  private Mechanism2d mechanism;
  private MechanismRoot2d root;
  private ArrayList<MechanismLigament2d> rollerLigament = new ArrayList<MechanismLigament2d>();

  public Slot0Configs getSlot0() {
    if (slot0 == null) {
      slot0 = simMotor.getSlot0Configuration();
    }
    return slot0;
  }

  public Slot1Configs getSlot1() {
    if (slot1 == null) {
      slot1 = simMotor.getSlot1Configuration();
    }
    return slot1;
  }

  public Slot2Configs getSlot2() {
    if (slot2 == null) {
      slot2 = simMotor.getSlot2Configuration();
    }
    return slot2;
  }
}
