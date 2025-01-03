package frc.robot.commands.base;

import frc.robot.constants.enums.LedModes;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.minor.TagalongLedSection;
import java.util.Map;
import java.util.function.Supplier;

public class LedVariableSpeedCommand extends LedCommand {
  private double _speed;
  private Supplier<Double> _speedSupplier;
  public LedVariableSpeedCommand(
      Leds leds, Map<Integer, LedModes> animationMap, Supplier<Double> speedFinder
  ) {
    super(leds, animationMap);
    _speedSupplier = speedFinder;
  }

  public LedVariableSpeedCommand(Leds leds, LedModes animation, Supplier<Double> speedFinder) {
    super(leds, animation);
    _speedSupplier = speedFinder;
  }

  @Override
  public void execute() {
    _speed = _speedSupplier.get();
    for (int i : _indices) {
      if (_lockHeld[i] || _leds.acquire(i)) {
        _lockHeld[i] = true;
        _leds._sections[i]._animations[_ledModes[i].ordinal()].setSpeed(_speed);
        // _leds.unsafeSetLEDMode(_ledModes[i], i);
      }
    }
  }
}
