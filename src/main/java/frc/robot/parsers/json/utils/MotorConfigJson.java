package frc.robot.parsers.json.utils;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;

public class MotorConfigJson {
  public PIDSGVAConstantsJson slot0PIDConstants = new PIDSGVAConstantsJson();
  public PIDSGVAConstantsJson slot1PIDConstants = new PIDSGVAConstantsJson();
  public PIDSGVAConstantsJson slot2PIDConstants = new PIDSGVAConstantsJson();

  public Slot0Configs getSlot0Configuration() {
    if (slot0PIDConstants == null) {
      return new Slot0Configs();
    }
    return slot0PIDConstants.getPIDSGVAConstants().toCTRESlot0Configuration();
  }
  public Slot1Configs getSlot1Configuration() {
    if (slot1PIDConstants == null) {
      return new Slot1Configs();
    }
    return slot1PIDConstants.getPIDSGVAConstants().toCTRESlot1Configuration();
  }
  public Slot2Configs getSlot2Configuration() {
    if (slot2PIDConstants == null) {
      return new Slot2Configs();
    }
    return slot2PIDConstants.getPIDSGVAConstants().toCTRESlot2Configuration();
  }
}
