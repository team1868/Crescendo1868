package frc.robot.parsers.json.utils;

import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;

public class LedConfJson {
  public DeviceJson candle;

  public int[] sectionLengths;

  public boolean statusLedOffWhenActive = true;
  public boolean disableWhenLOS = true;
  public String ledStripType = "rgb";
  public double brightness = 0.6;
  public boolean enable5V = true;
  public String vBatOutputMode = "off";

  private int stripLength = -1;
  private CANdleConfiguration candleConfig = null;
  private LEDStripType stripType = null;
  private VBatOutputMode vBatOutput = null;
  private int[] stripSections = null;

  public CANdleConfiguration getCANdleConfig() {
    if (candleConfig == null) {
      candleConfig = new CANdleConfiguration();
      candleConfig.statusLedOffWhenActive = statusLedOffWhenActive;
      candleConfig.disableWhenLOS = disableWhenLOS;
      candleConfig.stripType = getLEDStripType();
      candleConfig.brightnessScalar = brightness;
      candleConfig.v5Enabled = enable5V;
      candleConfig.vBatOutputMode = VBatOutputMode.Off;
    }
    return candleConfig;
  }

  public LEDStripType getLEDStripType() {
    if (stripType == null) {
      stripType = LEDStripType.valueOf(ledStripType.toUpperCase());
    }
    return stripType;
  }

  public VBatOutputMode getVBatOutputMode() {
    if (vBatOutput == null) {
      vBatOutput = VBatOutputMode.valueOf(vBatOutputMode.toUpperCase());
    }
    return vBatOutput;
  }

  public int getStripLength() {
    if (stripLength <= 0) {
      stripLength = 0;
      for (int i : sectionLengths) {
        stripLength += i;
      }
    }
    return stripLength;
  }

  public int[] getStripSections() {
    if (stripSections == null) {
      stripSections = new int[sectionLengths.length + 1];
      for (int i = 1; i < stripSections.length; i++) {
        stripSections[i] = stripSections[i - 1] + sectionLengths[i - 1];
      }
    }
    return stripSections;
  }
}
