package frc.robot.util.sim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.shooter.ShooterConstants;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

public class SimProjectile {
  public static GamePieceProjectile getProjectile(
      Translation2d drivePose, ChassisSpeeds fieldRelativeSpeeds, LinearVelocity launchVelocity) {
    return new RebuiltFuelOnFly(
            drivePose,
            ShooterConstants.shooterInitalPosition.getTranslation().toTranslation2d(),
            fieldRelativeSpeeds,
            drivePose
                .getAngle()
                .plus(ShooterConstants.shooterInitalPosition.getRotation().toRotation2d()),
            ShooterConstants.shooterInitalPosition.getMeasureZ(),
            launchVelocity,
            ShooterConstants.shooterInitalPosition.getRotation().getMeasureY())
        .withProjectileTrajectoryDisplayCallBack(
            // Callback for when the note will eventually hit the target (no target configured)
            (pose3ds) ->
                Logger.recordOutput(
                    "Shooter/FuelProjectileSuccessfulShot", pose3ds.toArray(Pose3d[]::new)),
            (pose3ds) ->
                Logger.recordOutput(
                    "Shooter/FuelProjectileUnsuccessfulShot", pose3ds.toArray(Pose3d[]::new)));
  }
}
