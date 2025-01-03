package frc.robot.commands.base;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.enums.LedModes;
import frc.robot.subsystems.Leds;

public class CrescendoTeleopLedCommand extends Command {
  private final Leds _leds;
  private final LedModes[][] _baseLedModes;

  public CrescendoTeleopLedCommand(
      Leds leds, LedModes initialModeNoPiece, LedModes initialModePiece
  ) {
    _leds = leds;
    _baseLedModes = new LedModes[_leds.getNumSections()][2];
    for (int i = 0; i < _baseLedModes.length; i++) {
      _baseLedModes[i][0] = initialModeNoPiece;
      _baseLedModes[i][1] = initialModePiece;
    }
    addRequirements(leds);
    runsWhenDisabled();
  }

  @Override
  public void execute() {
    updateFront();
    updateBack();
    updateLeftRight();
    // _leds.attemptToRestoreBase(_baseLedModes);
  }

  private void updateFront() {}
  private void updateBack() {}
  private void updateLeftRight() {}
}
