package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.ShooterCalculations;
import frc.robot.util.ShooterCalculations.ShooterControls;

public class ShooterCommands {
    public static class teleopShoot extends Command {
        private final Shooter shooter;
        private final Drive drive;
        private final DoubleSupplier ySupplier;
        private final DoubleSupplier xSupplier;

        private Rotation2d driveYawTarget;

        public teleopShoot(Drive drive, Shooter shooter,  DoubleSupplier xSupplier, DoubleSupplier ySupplier){
            this.drive = drive;
            this.shooter = shooter;
            this.xSupplier = xSupplier;
            this.ySupplier =  ySupplier;

            addRequirements(drive, shooter);
        }

        @Override
        public void execute() {
            ShooterControls controls = ShooterCalculations.getOptimizedShooterControls(drive.getPose(), drive.getChassisSpeeds(), shooter.getPivotAngle(), shooter.getFlywheelVelocity());
            shooter.setShooterControls(controls);

            Logger.recordOutput("Shooter Auto Aim/Drive Angle Setpoint", controls.driveYaw());
            driveYawTarget = controls.driveYaw();
        }

        @Override
        public void initialize() {
            DriveCommands.joystickDriveAtAngle(drive, xSupplier, ySupplier, this::getDriveYaw);
        }

        private Rotation2d getDriveYaw(){
            return driveYawTarget;
        }
        

        
    }
}
