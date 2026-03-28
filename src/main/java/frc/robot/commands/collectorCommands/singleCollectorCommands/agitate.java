// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collectorCommands.singleCollectorCommands;

import java.security.SignedObject;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.singleCollector.SingleCollector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class agitate extends Command {
  SingleCollector collector;
  Timer time;
  /** Creates a new agitate. */
  public agitate(SingleCollector collector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collector = collector;
    time = new Timer();
    addRequirements(collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.reset();
    time.start();
    collector.getSingleCollectorIO().goUp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(time.hasElapsed(.5)) {
      collector.getSingleCollectorIO().goDown();
    } else {
      collector.getSingleCollectorIO().goUp();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collector.getSingleCollectorIO().goDown();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
