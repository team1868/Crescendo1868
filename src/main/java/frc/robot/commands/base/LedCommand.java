package frc.robot.commands.base;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.enums.LedModes;
import frc.robot.subsystems.Leds;
import java.util.Map;

public class LedCommand extends Command {
  protected final Leds _leds;
  protected final LedModes[] _ledModes;
  protected final boolean[] _lockHeld;
  protected final int[] _indices;

  public LedCommand(Leds leds, LedModes animation) {
    _leds = leds;
    _ledModes = new LedModes[_leds.getNumSections()];
    _lockHeld = new boolean[_leds.getNumSections()];
    _indices = new int[_leds.getNumSections()];
    for (int i = 0; i < _indices.length; i++) {
      _ledModes[i] = animation;
      _indices[i] = i;
    }

    runsWhenDisabled();
  }

  public LedCommand(Leds leds, Map<Integer, LedModes> animationMap) {
    _leds = leds;
    _ledModes = new LedModes[_leds.getNumSections()];
    _lockHeld = new boolean[_leds.getNumSections()];
    _indices = new int[animationMap.keySet().size()];
    int i = 0;
    for (Integer key : animationMap.keySet()) {
      _indices[i++] = key.intValue();
      _ledModes[key.intValue()] = animationMap.get(key);
    }

    runsWhenDisabled();
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    for (int i : _indices) {
      if (_leds.acquire(i)) {
        _lockHeld[i] = true;
        // _leds.unsafeSetLEDMode(_ledModes[i], i);
      }
    }
  }

  @Override
  public boolean isFinished() {
    // return false;
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    for (int i : _indices) {
      if (_lockHeld[i]) {
        _leds.release(i);
        _lockHeld[i] = false;
      }
    }
  }
}
