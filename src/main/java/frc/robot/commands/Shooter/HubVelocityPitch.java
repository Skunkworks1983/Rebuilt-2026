package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.ShooterCalculations;
import frc.robot.util.ShooterCalculations.ShooterControls;

public class HubVelocityPitch extends Command {
    private final Shooter shooter;
    private final Drive drive;

    public HubVelocityPitch(Shooter shooter, Drive drive) {
        this.shooter = shooter;
        this.drive = drive;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        ShooterControls controls = ShooterCalculations.getApexConstrainedShooterControls(
                drive.getPose(), drive.getChassisSpeeds());

        shooter.setShooterControls(controls);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
