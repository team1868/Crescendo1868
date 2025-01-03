package frc.robot.subsystems.minor;

import com.ctre.phoenix.led.Animation;
import frc.robot.constants.enums.LedModes;
import frc.robot.parsers.json.utils.LedConfJson;

public class TagalongLedSection {
  private LedConfJson _conf;

  public final int _section;
  public final int _sectionOffset;
  public final int _sectionLength;

  public final Animation[] _animations;

  public TagalongLedSection(int section, LedConfJson conf) {
    int[] stripSections = conf.getStripSections();
    _section = section;
    _conf = conf;
    _animations = initializeAnimations();

    assert (stripSections.length >= section + 1);
    _sectionOffset = stripSections[section];
    _sectionLength = stripSections[section + 1] - stripSections[section];
  }

  private Animation[] initializeAnimations() {
    Animation[] animations = new Animation[LedModes.values().length];
    Integer integerLength = Integer.valueOf(_sectionLength);
    Integer integerOffset = Integer.valueOf(_sectionOffset);

    for (LedModes mode : LedModes.values()) {
      animations[mode.ordinal()] = mode._animationSupplier.apply(integerLength, integerOffset);
    }

    return animations;
  }
}
