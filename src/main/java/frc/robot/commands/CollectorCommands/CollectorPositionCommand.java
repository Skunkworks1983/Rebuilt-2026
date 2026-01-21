// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CollectorCommands;

import static edu.wpi.first.units.Units.Revolutions;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.Collector;

/**  */
public class CollectorPositionCommand extends Command {
  Collector collector;

  PositionVoltage positionVoltage;
  TrapezoidProfile.State goal;
  TrapezoidProfile.State startPosition;
  Angle setAngle;

  final TrapezoidProfile trapazoidProfile=
      new TrapezoidProfile(
        new TrapezoidProfile.Constraints(0, 0));
  
  Timer time;
  /** Creates a new CollectorPositionCommand. */
  public CollectorPositionCommand(Collector collector, Angle setAngle) {
    collector = collector;
    setAngle = setAngle;

    addRequirements(collector);

    time = new Timer();
    time.stop();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.reset();
    time.start();

    Angle newSetPoint = setAngle;

    //OOOOOOOOOO French Revolutions
    goal = new TrapezoidProfile.State(newSetPoint.in(Revolutions), 0);
    positionVoltage = new PositionVoltage(0);
    startPosition = new TrapezoidProfile.State();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
