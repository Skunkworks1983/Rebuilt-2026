package frc.robot.commands.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ShooterSolverConstants;

import org.littletonrobotics.junction.Logger;

public class ShootPresetSetpoints extends Command {
  public enum TargetType {
    GOAL,
    TRANSFER
  }

  private final Shooter shooter;
  private final TargetType targetType;
  private final LoggedTunableNumber upperVelocitySetpointRPS;
  private final LoggedTunableNumber lowerVelocitySetpointRPS;
  private final LoggedTunableNumber pitchSetpointDegrees;

  public ShootPresetSetpoints(Shooter shooter, TargetType targetType) {
    this.shooter = shooter;
    this.targetType = targetType;

    if (targetType == TargetType.GOAL) {
      upperVelocitySetpointRPS =
          new LoggedTunableNumber(
              "Shooter/Upper Flywheel/Goal Velocity Default Setpoint [RPS]",
              ShooterConstants.defaultUpperFlywheelVelocityGoal.in(RotationsPerSecond));
      lowerVelocitySetpointRPS =
          new LoggedTunableNumber(
              "Shooter/Lower Flywheel/Goal Velocity Default Setpoint [RPS]",
              ShooterConstants.defaultLowerFlywheelVelocityGoal.in(RotationsPerSecond));
      pitchSetpointDegrees =
          new LoggedTunableNumber(
              "Shooter/Goal Pitch Default Setpoint [Degrees]",
              ShooterConstants.defaultPitchGoal.in(Degrees));
    } else {
      upperVelocitySetpointRPS =
          new LoggedTunableNumber(
              "Shooter/Upper Flywheel/Transfer Velocity Default Setpoint [RPS]",
              ShooterConstants.defaultVelocityTransfer.in(RotationsPerSecond));
      lowerVelocitySetpointRPS =
          new LoggedTunableNumber(
              "Shooter/Lower Flywheel/Transfer Velocity Default Setpoint [RPS]",
              ShooterConstants.defaultLowerFlywheelVelocityTransfer.in(RotationsPerSecond));
      pitchSetpointDegrees =
          new LoggedTunableNumber(
              "Shooter/Transfer Pitch Default Setpoint [Degrees]",
              ShooterConstants.defaultPitchTransfer.in(Degrees));
    }
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("Commands/Manual Shoot/Shoot preset setpoints", true);
    shooter.setShooterSetpoints(
        new Shooter.ShooterSetpoints(
            RotationsPerSecond.of(upperVelocitySetpointRPS.get()),
            RotationsPerSecond.of(lowerVelocitySetpointRPS.get()),
            Degrees.of(pitchSetpointDegrees.get())));
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
