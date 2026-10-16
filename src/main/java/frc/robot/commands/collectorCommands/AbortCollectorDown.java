// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collectorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.Collector;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AbortCollectorDown extends Command {
  Collector collector;
  /** Creates a new CollectorDown. */
  public AbortCollectorDown(Collector collector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collector = collector;
    addRequirements(collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.recordOutput("Commands/Collector/CollectorDown", true);
    collector.goDown();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (collector.getCurrent().baseUnitMagnitude() >= 15) {
      collector.goUp();
    }
    // System.out.println("CURRENT AMPS: " + collector.getCurrent().in(Amps));
    Logger.recordOutput("Seen motor currents", collector.getCurrent());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("Commands/Collector/CollectorDown", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
