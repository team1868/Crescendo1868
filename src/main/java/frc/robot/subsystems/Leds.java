package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.enums.LedColors;
import frc.robot.constants.enums.LedModes;
import frc.robot.parsers.LedParser;
import frc.robot.parsers.json.utils.LedConfJson;
import frc.robot.subsystems.minor.TagalongLedSection;
import frc.robot.utils.TagalongSubsystemBase;
import java.util.function.Supplier;

public class Leds extends TagalongSubsystemBase {
  public static final int WHITE = 0;
  public static final int BRIGHTNESS = 150;

  public final LedParser _ledParser;
  public final LedConfJson _ledConf;

  public CANdle _candle;
  private final LedModes[] _currentLedModes;
  public final TagalongLedSection[] _sections;
  public final boolean[] _locks;
  private final Supplier<Boolean> _hasPieceInChute;

  public Leds(String filePath, Supplier<Boolean> hasPieceInChute) {
    this(
        filePath == null ? null : new LedParser(Filesystem.getDeployDirectory(), filePath),
        hasPieceInChute
    );
  }

  public Leds(LedParser ledParser, Supplier<Boolean> hasPieceInChute) {
    super(ledParser);
    _ledParser = ledParser;
    if (_configuredDisable) {
      _ledConf = null;
      _currentLedModes = new LedModes[0];
      _sections = new TagalongLedSection[0];
      _locks = new boolean[0];
      _hasPieceInChute = () -> false;
      return;
    }

    _hasPieceInChute = hasPieceInChute;
    _ledConf = ledParser._ledConf;
    _currentLedModes = new LedModes[_ledConf.sectionLengths.length];
    _sections = new TagalongLedSection[_ledConf.sectionLengths.length];
    _locks = new boolean[_ledConf.sectionLengths.length];

    for (int i = 0; i < _ledConf.sectionLengths.length; i++) {
      _sections[i] = new TagalongLedSection(i, _ledConf);
    }

    _candle = _ledConf.candle.getCANdle();
    _candle.configFactoryDefault();
    _candle.configAllSettings(_ledConf.getCANdleConfig(), 100);
  }

  @Override
  public boolean checkInitStatus() {
    return true;
  }

  @Override
  public void periodic() {
    if (_isSubsystemDisabled) {
      return;
    }
    if (_currentLedModes[0] != null)
      _candle.animate(_currentLedModes[0]._fullAnimation);
  }

  public int getNumSections() {
    return _sections.length;
  }

  public void unsafeSetLEDMode(LedModes ledMode, int section) {
    if ((!_isSubsystemDisabled || (ledMode != null && _currentLedModes[section] != ledMode))
        && section == 0) {
      _currentLedModes[section] = ledMode;
      // _candle.animate(_sections[section]._animations[ledMode.ordinal()], section);
      _candle.animate(ledMode._fullAnimation);
    }
  }

  public void attemptToRestoreBase(LedModes[][] base) {
    if (!_isSubsystemDisabled) {
      for (int i = 0; i < base.length; i++) {
        int hasPiece = _hasPieceInChute.get() ? 1 : 0;
        if (base[i][hasPiece] != _currentLedModes[i] && !_locks[i]) {
          _candle.animate(_sections[i]._animations[base[i][hasPiece].ordinal()], i);
        }
      }
    }
  }

  public boolean acquire(int section) {
    if (!(_isSubsystemDisabled || _locks[section])) {
      _locks[section] = true;
      return true;
    }
    return false;
  }

  public void release(int section) {
    if (!_isSubsystemDisabled) {
      _locks[section] = false;
    }
  }

  // TODO: Handle homogenous animation
  // public void setLEDMode(LedModes ledMode) {
  //   for (int i = 0; i < _currentLedModes.length; i++) {
  //     _currentLedModes[i] = ledMode;
  //   }

  //   _candle.
  // }
}
