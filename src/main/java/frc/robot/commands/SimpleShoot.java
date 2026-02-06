package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.simpleShooter.Shooter;
import frc.robot.subsystems.simpleShooter.ShooterConstants;
import frc.robot.util.LoggedTunableNumber;

public class SimpleShoot {
  public static Command simpleShoot(Shooter shooter) {
    LoggedTunableNumber forwardFlywheelVelocity =
        new LoggedTunableNumber(
            "Shooter/ForwardFlywheelVelocitySetpointRotPerSec",
            ShooterConstants.defaultForwardFlywheelVelocity.in(RotationsPerSecond));
    LoggedTunableNumber backFlywheelVelocity =
        new LoggedTunableNumber(
            "Shooter/BackFlywheelVelocitySetpointRotPerSec",
            ShooterConstants.defaultBackFlywheelVelocity.in(RotationsPerSecond));
    return Commands.run(
        () -> {
          shooter.setShooterControls(
              RotationsPerSecond.of(forwardFlywheelVelocity.get()),
              RotationsPerSecond.of(backFlywheelVelocity.get()));
        },
        shooter).finallyDo(() -> {shooter.setShooterControls(RotationsPerSecond.of(0), RotationsPerSecond.of(0));});
  }
}
