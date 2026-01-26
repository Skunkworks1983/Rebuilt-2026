// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collectorCommands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.Collector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CollectorUpPos extends Command {
  private Collector collector;
  private String downCollector;
  private Angle upAngle;

  /**
   * The collector state in which both collectors are up 🚀
   *
   * @param
   */
  public CollectorUpPos(Collector collector, Angle upAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collector = collector;
    addRequirements(collector);
    downCollector = null;
    this.upAngle = upAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    downCollector = collector.getDownCollector();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (downCollector) {
      case "NEITHER":
        collector.stopSound();
        if (collector.getRightCollector().getPosition().in(Degrees) < 0.0
            && collector.getRightCollector().getPosition().in(Degrees)
                < upAngle.in(Degrees)) { // TODO: replace 0.0 w/ collector up position
          collector.setDownCollector(
              "BOTH!!!"); // if this is returned and we didnt intend for it, it is an EMERGENCY!!
        } else if (collector.getLeftCollector().getPosition().in(Degrees) < upAngle.in(Degrees)) {
          collector.setDownCollector("LEFT");
        } else if (collector.getRightCollector().getPosition().in(Degrees) < upAngle.in(Degrees)) {
          collector.setDownCollector("RIGHT");
        } else {
          collector.setDownCollector("NEITHER");
        }
        break;

      case "LEFT":
        collector.stopSound();
        collector.getLeftCollector().setCollectorPos(upAngle);
        if (collector.getLeftCollector().getPosition().in(Degrees) >= upAngle.in(Degrees)) {
          collector.setDownCollector("NEITHER");
        }
        break;

      case "RIGHT":
        collector.stopSound();
        collector.getRightCollector().setCollectorPos(upAngle);
        if (collector.getRightCollector().getPosition().in(Degrees) >= upAngle.in(Degrees)) {
          collector.setDownCollector("NEITHER");
        }
        break;

      case "BOTH":
        collector.getRightCollector().setCollectorPos(upAngle);
        collector.getLeftCollector().setCollectorPos(upAngle);
        System.out.println("EMERGENCY!!! EMERGENCY!!! BOTH COLLECTORS DOWN🚨🚨🚨🚨");
        if (!collector.isSoundPlaying()) {
          collector.playSound("src/main/deploy/chrp/output.chrp");
        }
        if (collector.getRightCollector().getPosition().in(Degrees) >= upAngle.in(Degrees)
            && collector.getLeftCollector().getPosition().in(Degrees) >= upAngle.in(Degrees)) {
          collector.setDownCollector("NEITHER");
        }

      default:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
