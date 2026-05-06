package frc.robot.commands.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class ShootPresetSetpoints extends Command {
  public enum TargetType {
    GOAL,
    TRANSFER
  }

  private final Shooter shooter;
  private final LoggedTunableNumber upperVelocitySetpointRPS;
  private final LoggedTunableNumber pitchSetpointDeg;

  public ShootPresetSetpoints(Shooter shooter, TargetType targetType) {
    this.shooter = shooter;

    if (targetType == TargetType.GOAL) {
      upperVelocitySetpointRPS =
          new LoggedTunableNumber(
              "Shooter/Flywheel/Goal Velocity Default Setpoint [RPS]",
              ShooterConstants.defaultUpperFlywheelVelocityGoal.in(RotationsPerSecond));

      pitchSetpointDeg = new LoggedTunableNumber("Shooter/Pivot/Goal Default Pitch [Deg]", ShooterConstants.defaultPitchGoal.in(Degrees));
    } else {
      upperVelocitySetpointRPS =
          new LoggedTunableNumber(
              "Shooter/Flywheel/Transfer Velocity Default Setpoint [RPS]",
              ShooterConstants.defaultVelocityTransferFar.in(RotationsPerSecond));

      pitchSetpointDeg =new LoggedTunableNumber("Shooter/Pivot/Transfer Default Pitch [Deg]", ShooterConstants.defaultPitchTransferFar.in(Degrees));
    }
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("Commands/Manual Shoot/Shoot preset setpoints", true);
    shooter.setShooterSetpoints(
        new Shooter.ShooterSetpoints(
            RotationsPerSecond.of(upperVelocitySetpointRPS.get()), Degrees.of(pitchSetpointDeg.get())));
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopFlywheels();
    shooter.setPivotHome();
    Logger.recordOutput("Commands/Manual Shoot/Shoot preset setpoints", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
