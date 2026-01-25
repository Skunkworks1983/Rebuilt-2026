package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ShooterCalculations {
    public record shooterControls(
            LinearVelocity shooterVelocity, Angle shooterPitch, Angle driveYaw) {
    };

    public shooterControls getShooterControlsGoal(ChassisSpeeds speeds, Pose2d odometryPose,
            LinearVelocity shooterLinearVelocity, Distance xDistance, Distance zDistance) {
        Translation2d target = DriverStation.getAlliance().get() == Alliance.Red
                ? FieldConstants.redGoal
                : FieldConstants.blueGoal;

        Rotation2d rawYaw = target.minus(odometryPose.getTranslation()).getAngle();
                
        LinearVelocity shooterVelocitySquared = MetersPerSecond
                .of(Math.pow(shooterLinearVelocity.in(MetersPerSecond), 2));
        LinearVelocity shooterVelocityQuartic = MetersPerSecond
                .of(Math.pow(shooterLinearVelocity.in(MetersPerSecond), 4));
        Distance xDistanceSquared = Meters.of(Math.pow(xDistance.in(Meters), 2));

        double gravity = 9.81;

        Angle shooterPitch = Degrees.of(Math.atan2(
                shooterVelocityQuartic.in(MetersPerSecond) - Math.sqrt(shooterVelocityQuartic.in(MetersPerSecond)
                        - (gravity * ((gravity * xDistanceSquared.in(Meters))
                                + (2 * zDistance.in(Meters) * shooterVelocitySquared.in(MetersPerSecond))))),
                gravity * xDistance.in(Meters)));

        return null;
    }
}
