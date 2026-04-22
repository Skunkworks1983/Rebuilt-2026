package frc.robot.commands.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterSetpoints;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class ShootPresetSetpoints extends Command {
  public enum TargetType {
    RegularShot,
    SafeShot
  }

  TargetType targetType;

  private final LoggedTunableNumber regularFlywheelVelocity;
  private final LoggedTunableNumber demoFlyWheelVelocity;
  private final LoggedTunableNumber regularPitch;
  private final LoggedTunableNumber demoPitch;

  private final Shooter shooter;
  LoggedNetworkBoolean useDemoMode;

  public ShootPresetSetpoints(Shooter shooter) {
    this.shooter = shooter;

    regularFlywheelVelocity =
        new LoggedTunableNumber(
            "Shooter/Flywheel/Regular Velocity Default Setpoint [RPS]",
            ShooterConstants.deafultRegularVelocity.in(RotationsPerSecond));

    demoFlyWheelVelocity =
        new LoggedTunableNumber(
            "Shooter/Flywheel/Demo Velocity Default Setpoint [RPS]",
            ShooterConstants.defaultDemoVelocity.in(RotationsPerSecond));

    regularPitch =
        new LoggedTunableNumber(
            "Shooter/Pitch/Regular Default Setpoint [deg]",
            ShooterConstants.defaultRegularPitch.in(Degrees));

    demoPitch =
        new LoggedTunableNumber(
            "Shooter/Pitch/Demo Default Setpoint [deg]",
            ShooterConstants.defaultDemoPitch.in(Degrees));
    useDemoMode = new LoggedNetworkBoolean("Shooter/Use Demo Mode Setpoints");
    useDemoMode.setDefault(false);
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("Commands/Manual Shoot/Shoot preset setpoints", true);

    ShooterSetpoints setpoints;
    if (!useDemoMode.get()) {
      setpoints =
          new ShooterSetpoints(
              RotationsPerSecond.of(regularFlywheelVelocity.get()), Degrees.of(regularPitch.get()));
    } else {
      setpoints =
          new ShooterSetpoints(
              RotationsPerSecond.of(demoFlyWheelVelocity.get()), Degrees.of(demoPitch.get()));
    }
    shooter.setShooterSetpoints(setpoints);
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
