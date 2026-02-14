package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.FieldConstants.HubCorners;
import org.littletonrobotics.junction.Logger;

/**
 * Utility class for calculating shooter setpoints based on field geometry and robot state. Includes
 * both analytical and iterative solver methods.
 *
 * @author Eddy Walsh
 */
public class ShooterCalculations {
  /**
   * Data record representing the calculated control outputs for the shooter subsystem.
   *
   * @param shooterVelocity Target flywheel angular velocity.
   * @param shooterPitch Target pivot angle.
   * @param driveYaw Target robot chassis heading.
   * @param possibleShot Whether the shot is physically possible given the constraints.
   */
  public record ShooterControls(
      AngularVelocity shooterVelocity,
      Angle shooterPitch,
      Rotation2d driveYaw,
      boolean possibleShot) {}

  /**
   * Calculates shooter controls using an apex-constrained ballistic model. Assumes the shot peaks
   * at a specific height clearance above the goal.
   *
   * @param odometryPose Current robot pose.
   * @param chassisSpeeds Current robot chassis speeds.
   * @param gyroRate Current robot gyro rotation rate.
   * @return Calculated {@link ShooterControls}.
   */
  public static ShooterControls getApexConstrainedShooterControls(
      Pose2d odometryPose, ChassisSpeeds robotRelativeSpeeds) {
    Translation3d goal =
        DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
            ? FieldConstants.redGoal
            : FieldConstants.blueGoal;

    Rotation2d currentRobotRotation = odometryPose.getRotation();

    Translation2d robotLinearVelocityField =
        new Translation2d(
                robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond)
            .rotateBy(currentRobotRotation);

    Pose2d releasePose =
        odometryPose.exp(
            new Twist2d(
                robotRelativeSpeeds.vxMetersPerSecond
                    * ShooterSolverConstants.defaultShotLatencySec,
                robotRelativeSpeeds.vyMetersPerSecond
                    * ShooterSolverConstants.defaultShotLatencySec,
                robotRelativeSpeeds.omegaRadiansPerSecond
                    * ShooterSolverConstants.defaultShotLatencySec));
    Translation2d shooterOffsetField =
        ShooterConstants.shooterInitalPosition
            .getTranslation()
            .toTranslation2d()
            .rotateBy(releasePose.getRotation());

    Translation2d releaseShooterPosField = releasePose.getTranslation().plus(shooterOffsetField);

    Translation2d targetVector = goal.toTranslation2d().minus(releaseShooterPosField);
    double trueDistance = targetVector.getNorm();

    // Simple drag approximation
    double dragMultiplier = 1.0 + (ShooterSolverConstants.drag * trueDistance);
    dragMultiplier = MathUtil.clamp(dragMultiplier, 1.0, ShooterSolverConstants.maxDragMultiplier);
    double distWithDrag = trueDistance * dragMultiplier;

    double relativeGoalZ = goal.getZ() - ShooterConstants.shooterInitalPosition.getZ();

    // Apex height calculation
    double heightTrajectoryPeak =
        relativeGoalZ + ShooterSolverConstants.hubHeightClearance.in(Meters);
    heightTrajectoryPeak = Math.max(heightTrajectoryPeak, 0.1);

    double launchVelocityZ = Math.sqrt(2.0 * ShooterSolverConstants.gravity * heightTrajectoryPeak);

    double timeUp = launchVelocityZ / ShooterSolverConstants.gravity;
    double heightDown = Math.max(heightTrajectoryPeak - relativeGoalZ, 0.0);
    double timeDescend = Math.sqrt(2.0 * heightDown / ShooterSolverConstants.gravity);

    double totalTime = timeUp + timeDescend;
    totalTime = Math.max(totalTime, 1e-4);

    // Speed required if the robot were standing still
    double requiredHorizontalSpeed = distWithDrag / totalTime;

    // Angle
    Rotation2d angleToGoal = targetVector.getAngle();
    Translation2d stationaryLaunchVectorField =
        new Translation2d(requiredHorizontalSpeed, angleToGoal);

    // Average field velocity during flight
    Translation2d chassisVelField =
        new Translation2d(
                robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond)
            .rotateBy(releasePose.getRotation());

    double omega = robotRelativeSpeeds.omegaRadiansPerSecond;

    Translation2d tangentialVelField =
        new Translation2d(-omega * shooterOffsetField.getY(), omega * shooterOffsetField.getX());

    Translation2d shooterVelField = chassisVelField.plus(tangentialVelField);

    // 3) Compensate using instantaneous shooter velocity at release
    Translation2d effectiveLaunchVectorField = stationaryLaunchVectorField.minus(shooterVelField);

    Rotation2d finalDriveYaw = effectiveLaunchVectorField.getAngle();

    double effectiveHorizontalSpeed = effectiveLaunchVectorField.getNorm();
    double ballExitSpeed = Math.hypot(effectiveHorizontalSpeed, launchVelocityZ);
    double flywheelSurfaceSpeed = ballExitSpeed / ShooterSolverConstants.flywheelEfficiency;

    double finalPitch = Math.atan2(launchVelocityZ, effectiveHorizontalSpeed);

    // DEBUG LOGGING
    // TODO: Remove/Reduce logging (esp because this can be gotten from log replay)
    Translation2d leadShiftField = robotLinearVelocityField.times(totalTime);

    Translation3d virtualGoal =
        new Translation3d(
            goal.getX() - leadShiftField.getX(), goal.getY() - leadShiftField.getY(), goal.getZ());

    Translation2d virtualTargetVector = virtualGoal.toTranslation2d().minus(releaseShooterPosField);

    Logger.recordOutput("Shooter/Goal/Real", goal);
    Logger.recordOutput("Shooter/Goal/Virtual", virtualGoal);
    Logger.recordOutput("Shooter/Goal/LeadShiftXY", leadShiftField);
    Logger.recordOutput("Shooter/TargetVector/Real", targetVector);
    Logger.recordOutput("Shooter/TargetVector/Virtual", virtualTargetVector);

    // Should be ~0
    Translation2d shouldMatchStationary = effectiveLaunchVectorField.plus(robotLinearVelocityField);
    Logger.recordOutput(
        "Shooter/StationaryMinusReconstructedError",
        shouldMatchStationary.minus(stationaryLaunchVectorField).getNorm());

    return new ShooterControls(
        RadiansPerSecond.of(flywheelSurfaceSpeed / ShooterConstants.flywheelRadius.in(Meters)),
        Radians.of(finalPitch),
        finalDriveYaw,
        true);
  }

  /**
   * Calculates shooter controls for the "transfer" shot (shooting into the alliance zone from the
   * transfer zone). Includes logic to avoid hitting the hub structure.
   *
   * <p>It is attempted to first calculate shooter pitch from the current velocity (beause we can
   * change pitch faster than velocity and it uses less amperage) If this fails, we set pitch to be
   * the midpoint of the possible pitch range, and caluclate needed flywheel velocity (this ensures
   * that we minimize having to change flywheel velocity)
   *
   * @param speeds Current robot chassis speeds.
   * @param odometryPose Current robot pose.
   * @param flywheelLinearVelocity Current flywheel linear velocity
   * @return Calculated {@link ShooterControls}.
   */
  public static ShooterControls getShooterControlsTransfer(
      ChassisSpeeds speeds, Pose2d odometryPose, LinearVelocity flywheelLinearVelocity) {
    Translation2d target =
        DriverStation.getAlliance().get() == Alliance.Red
            ? FieldConstants.redFuelTransferTarget
            : FieldConstants.blueFuelTransferTarget;

    HubCorners hubCorners =
        DriverStation.getAlliance().get() == Alliance.Red
            ? FieldConstants.redHubCorners
            : FieldConstants.blueHubCorners;
    Translation2d targetVector = target.minus(odometryPose.getTranslation());
    Logger.recordOutput("TransferShot/TargetVector", targetVector);
    Logger.recordOutput("TransferShot/Target", target);
    Rotation2d rawYaw = targetVector.getAngle();
    Distance distanceToTarget = Meters.of(targetVector.getNorm());

    Rotation2d finalYaw = rawYaw;

    // PART 1: YAW/ACTUAL TARGET CALCULATER (Hub avoidance)
    Translation2d[] corners =
        new Translation2d[] {
          hubCorners.getAllianceSideLeft(),
          hubCorners.getAllianceSideRight(),
          hubCorners.getNeutralSideLeft(),
          hubCorners.getNeutralSideRight()
        };

    Angle hubFurthestCCWAngle = Radians.of(-Double.MAX_VALUE); // Most positive (CCW) angle to hub
    Angle hubFurthestCWAngle = Radians.of(Double.MAX_VALUE); // Most negative (CW) angle to hub
    Distance minDistToHub = Meters.of(Double.MAX_VALUE);

    for (Translation2d corner : corners) {
      Translation2d robotToCorner = corner.minus(odometryPose.getTranslation());

      Distance distToCorner = Meters.of(robotToCorner.getNorm());
      if (distToCorner.lt(minDistToHub)) {
        minDistToHub = distToCorner;
      }

      Rotation2d angleToCorner =
          robotToCorner.getAngle().minus(rawYaw); // Using Rotation2d to handle angle wrapping

      Angle angleToCornerMeasure = Radians.of(angleToCorner.getRadians());

      if (angleToCornerMeasure.gt(hubFurthestCCWAngle)) hubFurthestCCWAngle = angleToCornerMeasure;
      if (angleToCornerMeasure.lt(hubFurthestCWAngle)) hubFurthestCWAngle = angleToCornerMeasure;
    }

    // 5. Collision Detection & Avoidance Logic
    boolean hubIsCloser = minDistToHub.lt(distanceToTarget);
    boolean lineOfSightBlocked =
        hubFurthestCWAngle.lt(Radians.of(0)) && hubFurthestCCWAngle.gt(Radians.of(0));

    if (hubIsCloser && lineOfSightBlocked) {
      Angle rightCandidate =
          hubFurthestCWAngle.minus(ShooterSolverConstants.transferShotSafetyMargin);
      Angle leftCandidate =
          hubFurthestCCWAngle.plus(ShooterSolverConstants.transferShotSafetyMargin);

      // Pick the smaller magnitude adjustment
      if (Math.abs(rightCandidate.in(Radians)) <= Math.abs(leftCandidate.in(Radians))) {
        finalYaw = rawYaw.plus(Rotation2d.fromRadians(rightCandidate.in(Radians)));
      } else {
        finalYaw = rawYaw.plus(Rotation2d.fromRadians(leftCandidate.in(Radians)));
      }

      double cosYaw = finalYaw.getCos();
      double netTargetDx = target.getX() - odometryPose.getTranslation().getX();

      if (Math.abs(cosYaw) > 1e-3) {
        double distanceToNewTarget = netTargetDx / cosYaw;

        if (distanceToNewTarget > 0.0) { // intersection is in front of robot
          distanceToTarget = Meters.of(distanceToNewTarget);
        }
      }
    }

    // PART 2: BALLISTIC SOLVER
    // This solves the standard projectile range equation
    double distanceToTargetMeters = distanceToTarget.in(Meters);
    double currentFlywheelLienarVelocity = flywheelLinearVelocity.in(MetersPerSecond);
    Angle calculatedPitch = null;

    double calculatedFlywheelVelocity = currentFlywheelLienarVelocity;

    // Attempt 1: Solve for Shooter Pitch given Current Velocity
    double flywheelLinearVelocitySquared =
        currentFlywheelLienarVelocity * currentFlywheelLienarVelocity;
    double sinArgument =
        (ShooterSolverConstants.gravity * distanceToTargetMeters) / flywheelLinearVelocitySquared;

    // If sinArgument > 1, the target is out of range for this velocity
    if (Math.abs(sinArgument) <= 1.0) {
      double twoTheta = Math.asin(sinArgument);

      // This is the "Low Arc" (Direct shot)
      Angle lowArc = Radians.of(0.5 * twoTheta);

      // This is the "High Arc" (90 degrees - Low Arc)
      Angle highArc = Radians.of(Math.toRadians(90)).minus(lowArc);

      if (isValidPitch(lowArc)) {
        calculatedPitch = lowArc;
      } else if (isValidPitch(highArc)) {
        calculatedPitch = highArc;
      }
    }

    // Attempt 2: Solve for Flywheel Velocity given Midpoint Pitch
    if (calculatedPitch == null) {
      double minRad = ShooterConstants.minShooterPitch.in(Radians);
      double maxRad = ShooterConstants.maxShooterPitch.in(Radians);
      Angle midpointPitch = Radians.of((minRad + maxRad) / 2.0);

      double sinTwoTheta = Math.sin(2.0 * midpointPitch.in(Radians));

      if (Math.abs(sinTwoTheta) > 1e-6) { // Avoid divide by zero
        double requiredV =
            Math.sqrt((ShooterSolverConstants.gravity * distanceToTargetMeters) / sinTwoTheta);
        calculatedFlywheelVelocity = requiredV;
        calculatedPitch = midpointPitch;
      } else {
        // Should basically never happen unless pitch is 0 or 90
        calculatedFlywheelVelocity = 0;
        calculatedPitch = midpointPitch;
      }
    }
    return new ShooterControls(
        RadiansPerSecond.of(
            calculatedFlywheelVelocity / ShooterConstants.flywheelRadius.in(Meters)),
        calculatedPitch,
        finalYaw,
        true);
  }

  private static boolean isValidPitch(Angle pitch) {
    double pitchDeg = pitch.in(Degrees);
    return pitchDeg >= ShooterConstants.minShooterPitch.in(Degrees)
        && pitchDeg <= ShooterConstants.maxShooterPitch.in(Degrees);
  }
}
