package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterSetpoints;
import frc.robot.util.ShooterCalculations;
import frc.robot.util.ShooterCalculations.ShooterControls;
import org.littletonrobotics.junction.Logger;

public class NoYawShootTransfer extends Command {
  private final Shooter shooter;
  private final Drive drive;

  /**
   * Sets shooter to correct flywheel velocity and pivot pitch to shoot transfer without controlling
   * drive yaw
   *
   * <p>This is primarily for autonomous
   *
   * @param shooter The shooter..
   * @param drive The drivetrain....
   */
  public NoYawShootTransfer(Shooter shooter, Drive drive) {
    this.shooter = shooter;
    this.drive = drive;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("Commands/Auto Shoot/Drive Aimed Transfer No Yaw", true);
  }

  @Override
  public void execute() {
    ShooterControls controls =
        ShooterCalculations.getShooterControlsTransfer(
            drive.getChassisSpeeds(), drive.getPose(), shooter.getShooterForwardLinearVelocity());

    shooter.setShooterSetpoints(
        new ShooterSetpoints(controls.flywheelVelocity(), controls.shooterPitch()));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopFlywheels();
    shooter.setPivotHome();
    Logger.recordOutput("Commands/Auto Shoot/Drive Aimed Transfer No Yaw", false);
  }
}
