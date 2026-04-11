package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class NetworkTablesWait extends Command {
  Timer timer = new Timer();
  LoggedNetworkNumber logTimer;
  private double duration = 5.0;
  private final String name;

  public NetworkTablesWait(String name) {
    this.name = name;
    logTimer = new LoggedNetworkNumber("Network Wait/" + name + "/Wait Duration", 5.0);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("Commands/NetworkWait/" + name, true);
    duration = logTimer.get();

    timer.reset();
    timer.start();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(duration);
  }

  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("Commands/NetworkWait/" + name, false);
  }
}
