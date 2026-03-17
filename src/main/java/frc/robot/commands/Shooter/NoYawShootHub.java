package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterSetpoints;
import frc.robot.util.ShooterCalculations;
import frc.robot.util.ShooterCalculations.ShooterControls;
import org.littletonrobotics.junction.Logger;

public class NoYawShootHub extends Command {
  private final Shooter shooter;
  private final Drive drive;
  private final Trigger readyToShoot;
  private boolean commandStarted = false;

  /**
   * Sets shooter to correct flywheel velocity and pivot pitch to shoot into the hub without
   * controlling drive yaw
   *
   * <p>This is primarily for autonomous
   *
   * @param shooter The shooter..
   * @param drive The drivetrain....
   */
  public NoYawShootHub(Shooter shooter, Drive drive, Command indexerCommand) {
    this.shooter = shooter;
    this.drive = drive;
  }

  @Override
  public void initialize() {
    Logger.recordOutput("Commands/Auto Shoot/Drive Aimed Goal No Yaw", true);
    commandStarted = true;
  }

  @Override
  public void execute() {
    ShooterControls controls =
        ShooterCalculations.getApexConstrainedShooterControls(
            drive.getPose(), drive.getChassisSpeeds());

    shooter.setShooterSetpoints(
        new ShooterSetpoints(
            controls.upperFlywheelVelocity(),
            controls.lowerFlywheelVelocity(),
            controls.shooterPitch()));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopFlywheels();
    shooter.setPivotHome();
    Logger.recordOutput("Commands/Auto Shoot/Drive Aimed Goal No Yaw", false);
    commandStarted = false;
  }
}
