// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collectorCommands;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.Collector;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class agitate extends Command {
  Collector collector;
  Timer time;
  /** Creates a new agitate. */
  public agitate(Collector collector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collector = collector;
    time = new Timer();
    // addRequirements(collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.recordOutput("Commands/Collector/Agitate", true);
    time.reset();
    time.start();
    collector.setCollectorPos(Radians.of(collector.getConstants().CollectorHalfPos.get()), 0);
    collector.setCollectorIntakeVelocity(collector.getConstants().CollectorIntakeSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (time.hasElapsed(Seconds.of(1.5))) {
      time.reset();
    } else if (time.hasElapsed(Seconds.of(.75))) {
      collector.setCollectorPos(Radians.of(collector.getConstants().CollectorHalfPos.get()), 0);
    } else {
      collector.goDown();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("Commands/Collector/Agitate", false);
    collector.goDown();
    collector.setCollectorIntakeDutyCycle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
