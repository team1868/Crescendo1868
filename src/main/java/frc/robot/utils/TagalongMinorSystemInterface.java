package frc.robot.utils;

public interface TagalongMinorSystemInterface {
  /* -------- IO and config functions -------- */
  public void updateShuffleboard();
  public void configShuffleboard();

  // THESE MUST BE REGISTERED
  public void onEnable();
  public void onDisable();
  public void periodic();
  public void disabledPeriodic();
  public void simulationInit();
  public void simulationPeriodic();
}
