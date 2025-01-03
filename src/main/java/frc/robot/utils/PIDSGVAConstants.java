package frc.robot.utils;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;

public class PIDSGVAConstants extends PIDConstants {
  public final double s;
  public final double g;
  public final double v;
  public final double a;

  public PIDSGVAConstants(double p, double i, double d, double s, double g, double v, double a) {
    super(p, i, d);
    this.s = s;
    this.g = g;
    this.v = v;
    this.a = a;
  }

  public Slot0Configs toCTRESlot0Configuration() {
    Slot0Configs configured = new Slot0Configs();
    configured.kP = p;
    configured.kI = i;
    configured.kD = d;
    configured.kS = s;
    configured.kG = g;
    configured.kV = v;
    configured.kA = a;

    return configured;
  }
  public Slot1Configs toCTRESlot1Configuration() {
    Slot1Configs configured = new Slot1Configs();
    configured.kP = p;
    configured.kI = i;
    configured.kD = d;
    configured.kS = s;
    configured.kG = g;
    configured.kV = v;
    configured.kA = a;

    return configured;
  }
  public Slot2Configs toCTRESlot2Configuration() {
    Slot2Configs configured = new Slot2Configs();
    configured.kP = p;
    configured.kI = i;
    configured.kD = d;
    configured.kS = s;
    configured.kG = g;
    configured.kV = v;
    configured.kA = a;

    return configured;
  }
}
