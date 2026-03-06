package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
 * Utility class for calculating shooter setpoints based on field geometry and
 * robot state. Includes
 * both analytical and iterative solver methods.
 *
 * @author Eddy Walsh
 */
public class ShooterCalculations {
  /**
   * Data record representing the calculated control outputs for the shooter
   * subsystem.
   *
   * @param shooterVelocity Target flywheel angular velocity.
   * @param shooterPitch    Target pivot angle.
   * @param driveYaw        Target robot chassis heading.
   * @param possibleShot    Whether the shot is physically possible given the
   *                        constraints.
   */
  public record ShooterControls(
      AngularVelocity upperFlywheelVelocity,
      AngularVelocity lowerFlywheelVelocity,
      Angle shooterPitch,
      Rotation2d driveYaw,
      boolean possibleShot) {
  }

  // Launching Maps
  private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap = new InterpolatingTreeMap<>(
      InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap flywheelSpeedMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

  static {
    hoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0));
    hoodAngleMap.put(1.78, Rotation2d.fromDegrees(19.0));
    hoodAngleMap.put(2.17, Rotation2d.fromDegrees(24.0));
    hoodAngleMap.put(2.81, Rotation2d.fromDegrees(27.0));
    hoodAngleMap.put(3.82, Rotation2d.fromDegrees(29.0));
    hoodAngleMap.put(4.09, Rotation2d.fromDegrees(30.0));
    hoodAngleMap.put(4.40, Rotation2d.fromDegrees(31.0));
    hoodAngleMap.put(4.77, Rotation2d.fromDegrees(32.0));
    hoodAngleMap.put(5.57, Rotation2d.fromDegrees(32.0));
    hoodAngleMap.put(5.60, Rotation2d.fromDegrees(35.0));

    flywheelSpeedMap.put(1.34, 210.0);
    flywheelSpeedMap.put(1.78, 220.0);
    flywheelSpeedMap.put(2.17, 220.0);
    flywheelSpeedMap.put(2.81, 230.0);
    flywheelSpeedMap.put(3.82, 250.0);
    flywheelSpeedMap.put(4.09, 255.0);
    flywheelSpeedMap.put(4.40, 260.0);
    flywheelSpeedMap.put(4.77, 265.0);
    flywheelSpeedMap.put(5.57, 275.0);
    flywheelSpeedMap.put(5.60, 290.0);

    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);
  }

  /**
   * Calculates shooter controls using an apex-constrained ballistic model. It
   * forces the trajectory
   * to peak a certain height above the Hub, meaning that it must enter with a
   * downward trajectory
   *
   * <h2>Implementation Details</h2>
   *
   * <ol>
   * <li>Predicts the robot pose at ball release by extrapolating the current
   * chassis speeds
   * forward by shot latency.
   * <li>Computes the shooter’s field position at release and the horizontal
   * vector from that
   * point to the goal.
   * <li>Approximates drag by inflating distance
   * <li>Forces the trajectory to have an apex above the goal by a fixed clearance
   * and computes
   * the required vertical launch velocity and total flight time.
   * <li>Use the total flight time to determine the horizontal field-relative
   * launch speed
   * required to reach the goal.
   * <li>Subtracts the shooter’s instantaneous field-relative velocity
   * (translation and rotation)
   * from the desired launch vector to compensate for robot motion.
   * <li>Determines the commanded drive yaw, shooter pitch, and flywheel speed
   * from the
   * compensated launch velocity.
   * </ol>
   *
   * @param odometryPose  Current robot pose.
   * @param chassisSpeeds Current robot chassis speeds.
   * @param gyroRate      Current robot gyro rotation rate.
   * @return Calculated {@link ShooterControls}.
   */
  public static ShooterControls getApexConstrainedShooterControls(
      Pose2d odometryPose, ChassisSpeeds robotRelativeSpeeds) {
    Translation3d goal = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
        ? FieldConstants.redGoal
        : FieldConstants.blueGoal;

    Rotation2d currentRobotRotation = odometryPose.getRotation();

    Translation2d robotLinearVelocityField = new Translation2d(
        robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond)
        .rotateBy(currentRobotRotation);

    Pose2d releasePose = odometryPose.exp(
        new Twist2d(
            robotRelativeSpeeds.vxMetersPerSecond
                * ShooterSolverConstants.defaultShotLatencySec,
            robotRelativeSpeeds.vyMetersPerSecond
                * ShooterSolverConstants.defaultShotLatencySec,
            robotRelativeSpeeds.omegaRadiansPerSecond
                * ShooterSolverConstants.defaultShotLatencySec));
    Translation2d shooterOffsetField = ShooterConstants.shooterInitalPosition
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
    double currentClearance = ShooterSolverConstants.hubHeightClearance.in(Meters);
    if (distWithDrag < ShooterSolverConstants.closeShotDistance.in(Meters)) {
      // 1. Force a minimum horizontal distance so the pitch leans forward into the
      // goal.
      // This ensures requiredHorizontalSpeed never approaches 0.
      distWithDrag = ShooterSolverConstants.closeShotDistance.in(Meters);
      Logger.recordOutput("Shooter/Shooting Close", true);

      // 2. Squash the apex trajectory.
      // We don't need a high, looping arc when we are point-blank.
      // A clearance of 0.05m to 0.1m just gives it enough lift to clear the rim.
      currentClearance = currentClearance * ShooterSolverConstants.closeShotClearenceMultiplier.get();
    }

    // Apex height calculation
    double heightTrajectoryPeak = relativeGoalZ + currentClearance;
    heightTrajectoryPeak = Math.max(heightTrajectoryPeak, 0.1);

    double launchVelocityZ = Math.sqrt(2.0 * ShooterSolverConstants.gravity * heightTrajectoryPeak);

    // Calculate the time for the ball to follow trajectory
    double timeUp = launchVelocityZ / ShooterSolverConstants.gravity;
    double heightDown = Math.max(heightTrajectoryPeak - relativeGoalZ, 0.0);
    double timeDescend = Math.sqrt(2.0 * heightDown / ShooterSolverConstants.gravity);

    double totalTime = timeUp + timeDescend;
    totalTime = Math.max(totalTime, 1e-4);

    // Speed required if the robot were standing still
    double requiredHorizontalSpeed = distWithDrag / totalTime;

    // Angle to aim
    Rotation2d angleToGoal = targetVector.getAngle();
    Translation2d stationaryLaunchVectorField = new Translation2d(requiredHorizontalSpeed, angleToGoal);

    // Average field velocity during flight
    Translation2d chassisVelField = new Translation2d(
        robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond)
        .rotateBy(releasePose.getRotation());

    double omega = robotRelativeSpeeds.omegaRadiansPerSecond;

    Translation2d tangentialVelField = new Translation2d(-omega * shooterOffsetField.getY(),
        omega * shooterOffsetField.getX());

    Translation2d shooterVelField = chassisVelField.plus(tangentialVelField);

    // 3) Compensate using instantaneous shooter velocity at release
    Translation2d effectiveLaunchVectorField = stationaryLaunchVectorField.minus(shooterVelField);

    Rotation2d finalDriveYaw = effectiveLaunchVectorField.getAngle();

    double minPitchRadians = ShooterConstants.minShooterPitch.in(Radians);
    double maxPitchRadians = ShooterConstants.maxShooterPitch.in(Radians);

    double effectiveHorizontalSpeed = effectiveLaunchVectorField.getNorm();
    double ballExitSpeed = Math.hypot(effectiveHorizontalSpeed, launchVelocityZ);

    double finalPitch = Math.atan2(launchVelocityZ, effectiveHorizontalSpeed);
    double unconstrainedPitch = finalPitch;
    boolean possibleShot = true;

    // if (finalPitch < minPitchRadians || finalPitch > maxPitchRadians) {
    // return solveBallExitSpeedForDistanceAndAngle(
    // finalPitch, distWithDrag, relativeGoalZ, angleToGoal, shooterVelField);
    // }

    double flywheelSurfaceSpeed = ballExitSpeed / ShooterSolverConstants.flywheelEfficiency;

    // DEBUG LOGGING
    // TODO: Remove/Reduce logging (esp because this can be gotten from log replay)
    Translation2d leadShiftField = robotLinearVelocityField.times(totalTime);

    Translation3d virtualGoal = new Translation3d(
        goal.getX() - leadShiftField.getX(), goal.getY() - leadShiftField.getY(), goal.getZ());

    Translation2d virtualTargetVector = virtualGoal.toTranslation2d().minus(releaseShooterPosField);

    Logger.recordOutput("Shooter/Goal/Real", goal);
    Logger.recordOutput("Shooter/Goal/Virtual", virtualGoal);
    Logger.recordOutput("Shooter/Goal/LeadShiftXY", leadShiftField);
    Logger.recordOutput("Shooter/TargetVector/Real", targetVector);
    Logger.recordOutput("Shooter/TargetVector/Virtual", virtualTargetVector);
    Logger.recordOutput(
        "Shooter/ApexConstrained/PitchBeforeClampDeg", Math.toDegrees(unconstrainedPitch));
    Logger.recordOutput("Shooter/ApexConstrained/PitchAfterClampDeg", Math.toDegrees(finalPitch));
    Logger.recordOutput(
        "Shooter/ApexConstrained/PitchWasClamped",
        unconstrainedPitch < minPitchRadians || unconstrainedPitch > maxPitchRadians);
    Logger.recordOutput("Shooter/ApexConstrained/PossibleShot", possibleShot);

    // Should be ~0
    Translation2d shouldMatchStationary = effectiveLaunchVectorField.plus(robotLinearVelocityField);
    Logger.recordOutput(
        "Shooter/StationaryMinusReconstructedError",
        shouldMatchStationary.minus(stationaryLaunchVectorField).getNorm());

    return new ShooterControls(
        RadiansPerSecond.of(flywheelSurfaceSpeed / ShooterConstants.flywheelRadius.in(Meters)),
        RadiansPerSecond.of(
            (flywheelSurfaceSpeed / ShooterConstants.flywheelRadius.in(Meters))
                * ShooterSolverConstants.lowerFlywheelVelocityMultiplier.get()),
        Radians.of(finalPitch),
        finalDriveYaw,
        possibleShot);
  }

  private static ShooterControls solveBallExitSpeedForDistanceAndAngle(
      double finalPitch,
      double distWithDrag,
      double relativeGoalZ,
      Rotation2d angleToGoal,
      Translation2d shooterVelField) {

    double constrainedPitchRadians = MathUtil.clamp(
        finalPitch,
        ShooterConstants.minShooterPitch.in(Radians),
        ShooterConstants.maxShooterPitch.in(Radians));

    Translation2d stationaryLaunchVectorField = new Translation2d();
    Translation2d effectiveLaunchVectorField = new Translation2d();
    double launchVelocityZ = 0;

    // Binary search over time of flight (t) to find the exact exit speed needed
    double min_t = 0.1; // 100ms: Very fast, flat shot (lower bound)
    double max_t = 4.0; // 4 seconds: Very slow, high lofted shot (upper bound)
    double t = 1.0;

    for (int i = 0; i < 15; i++) { // Increased to 15 for better convergence
      t = (min_t + max_t) / 2.0;

      // 1. Calculate required field-relative velocities for this time of flight
      double reqHorizontalSpeed = distWithDrag / t;
      double reqLaunchZ = (relativeGoalZ + 0.5 * ShooterSolverConstants.gravity * t * t) / t;

      // 2. Create the field-relative launch vector (NOW USING POLAR CONSTRUCTOR)
      Translation2d testStationaryVectorField = new Translation2d(reqHorizontalSpeed, angleToGoal);

      // 3. Subtract robot velocity to get what the shooter actually needs to do
      Translation2d testEffectiveLaunchVectorField = testStationaryVectorField.minus(shooterVelField);

      double calcHorizontalSpeed = testEffectiveLaunchVectorField.getNorm();
      double calcPitch = Math.atan2(reqLaunchZ, calcHorizontalSpeed);

      // 4. Adjust binary search bounds
      if (calcPitch > constrainedPitchRadians) {
        max_t = t;
      } else {
        min_t = t;
      }

      // Store values
      stationaryLaunchVectorField = testStationaryVectorField;
      effectiveLaunchVectorField = testEffectiveLaunchVectorField;
      launchVelocityZ = reqLaunchZ;
    }

    // 5. Apply the converged results
    double effectiveHorizontalSpeed = effectiveLaunchVectorField.getNorm();
    finalPitch = constrainedPitchRadians;
    double ballExitSpeed = Math.hypot(effectiveHorizontalSpeed, launchVelocityZ);

    double flywheelSurfaceSpeed = ballExitSpeed / ShooterSolverConstants.flywheelEfficiency;
    double flywheelRadPerSec = flywheelSurfaceSpeed / ShooterConstants.flywheelRadius.in(Meters);

    // TODO: Consider adding Logger.recordOutput() here so you don't lose telemetry
    // when clamped!

    return new ShooterControls(
        RadiansPerSecond.of(flywheelRadPerSec),
        RadiansPerSecond.of(
            flywheelRadPerSec * ShooterSolverConstants.lowerFlywheelVelocityMultiplier.get()),
        Radians.of(finalPitch),
        effectiveLaunchVectorField.getAngle(),
        true);
  }

  /**
   * Calculates shooter controls for the "transfer" shot (shooting into the
   * alliance zone from the
   * transfer zone). Includes logic to avoid hitting the hub structure.
   *
   * <p>
   * It is attempted to first calculate shooter pitch from the current velocity
   * (because we can
   * change pitch faster than velocity and it uses less amperage). If this fails,
   * we set pitch to be
   * the midpoint of the possible pitch range, and calculate needed flywheel
   * velocity.
   *
   * @param speeds                 Current robot chassis speeds.
   * @param odometryPose           Current robot pose.
   * @param flywheelLinearVelocity Current flywheel surface linear velocity.
   * @return Calculated {@link ShooterControls}.
   */
  public static ShooterControls getShooterControlsTransfer(
      ChassisSpeeds speeds, Pose2d odometryPose, LinearVelocity flywheelLinearVelocity) {
    Translation2d target = DriverStation.getAlliance().get() == Alliance.Red
        ? FieldConstants.redFuelTransferTarget
        : FieldConstants.blueFuelTransferTarget;

    HubCorners hubCorners = DriverStation.getAlliance().get() == Alliance.Red
        ? FieldConstants.redHubCorners
        : FieldConstants.blueHubCorners;

    Translation2d targetVector = target.minus(odometryPose.getTranslation());
    Logger.recordOutput("TransferShot/TargetVector", targetVector);
    Logger.recordOutput("TransferShot/Target", target);

    Rotation2d rawYaw = targetVector.getAngle();
    Distance distanceToTarget = Meters.of(targetVector.getNorm());
    Rotation2d finalYaw = rawYaw;

    // PART 1: YAW / TARGET CALCULATOR (Hub avoidance)
    Translation2d[] corners = new Translation2d[] {
        hubCorners.getAllianceSideLeft(),
        hubCorners.getAllianceSideRight(),
        hubCorners.getNeutralSideLeft(),
        hubCorners.getNeutralSideRight()
    };

    Angle hubFurthestCCWAngle = Radians.of(-Double.MAX_VALUE);
    Angle hubFurthestCWAngle = Radians.of(Double.MAX_VALUE);
    Distance minDistToHub = Meters.of(Double.MAX_VALUE);

    for (Translation2d corner : corners) {
      Translation2d robotToCorner = corner.minus(odometryPose.getTranslation());

      Distance distToCorner = Meters.of(robotToCorner.getNorm());
      if (distToCorner.lt(minDistToHub)) {
        minDistToHub = distToCorner;
      }

      Rotation2d angleToCorner = robotToCorner.getAngle().minus(rawYaw);
      Angle angleToCornerMeasure = Radians.of(angleToCorner.getRadians());

      if (angleToCornerMeasure.gt(hubFurthestCCWAngle)) {
        hubFurthestCCWAngle = angleToCornerMeasure;
      }
      if (angleToCornerMeasure.lt(hubFurthestCWAngle)) {
        hubFurthestCWAngle = angleToCornerMeasure;
      }
    }

    boolean hubIsCloser = minDistToHub.lt(distanceToTarget);
    boolean lineOfSightBlocked = hubFurthestCWAngle.lt(Radians.of(0)) && hubFurthestCCWAngle.gt(Radians.of(0));

    if (hubIsCloser && lineOfSightBlocked) {
      Angle rightCandidate = hubFurthestCWAngle.minus(ShooterSolverConstants.transferShotSafetyMargin);
      Angle leftCandidate = hubFurthestCCWAngle.plus(ShooterSolverConstants.transferShotSafetyMargin);

      if (Math.abs(rightCandidate.in(Radians)) <= Math.abs(leftCandidate.in(Radians))) {
        finalYaw = rawYaw.plus(Rotation2d.fromRadians(rightCandidate.in(Radians)));
      } else {
        finalYaw = rawYaw.plus(Rotation2d.fromRadians(leftCandidate.in(Radians)));
      }

      double cosYaw = finalYaw.getCos();
      double netTargetDx = target.getX() - odometryPose.getTranslation().getX();

      if (Math.abs(cosYaw) > 1e-3) {
        double distanceToNewTarget = netTargetDx / cosYaw;
        if (distanceToNewTarget > 0.0) {
          distanceToTarget = Meters.of(distanceToNewTarget);
        }
      }
    }

    // PART 2: BALLISTIC SOLVER
    double distanceToTargetMeters = distanceToTarget.in(Meters);

    // Raw flywheel surface speed
    double currentFlywheelLinearVelocity = flywheelLinearVelocity.in(MetersPerSecond);

    // Effective projectile launch speed after efficiency losses
    double currentProjectileVelocity = currentFlywheelLinearVelocity * ShooterSolverConstants.flywheelEfficiency;

    Angle calculatedPitch = null;

    // This variable stores PROJECTILE velocity through the solve,
    // then gets converted back to flywheel velocity at the end.
    double calculatedProjectileVelocity = currentProjectileVelocity;

    // Attempt 1: Solve for Shooter Pitch given CURRENT PROJECTILE VELOCITY
    double projectileVelocitySquared = currentProjectileVelocity * currentProjectileVelocity;
    double sinArgument = (ShooterSolverConstants.gravity * distanceToTargetMeters) / projectileVelocitySquared;

    if (Math.abs(sinArgument) <= 1.0) {
      double twoTheta = Math.asin(sinArgument);

      Angle lowArc = Radians.of(0.5 * twoTheta);
      Angle highArc = Radians.of(Math.toRadians(90)).minus(lowArc);

      if (isValidPitch(lowArc)) {
        calculatedPitch = lowArc;
      } else if (isValidPitch(highArc)) {
        calculatedPitch = highArc;
      }
    }

    // Attempt 2: Solve for required PROJECTILE VELOCITY given midpoint pitch
    if (calculatedPitch == null) {
      double minRad = ShooterConstants.minShooterPitch.in(Radians);
      double maxRad = ShooterConstants.maxShooterPitch.in(Radians);
      Angle midpointPitch = Radians.of((minRad + maxRad) / 2.0);

      double sinTwoTheta = Math.sin(2.0 * midpointPitch.in(Radians));

      if (Math.abs(sinTwoTheta) > 1e-6) {
        double requiredProjectileVelocity = Math
            .sqrt((ShooterSolverConstants.gravity * distanceToTargetMeters) / sinTwoTheta);
        calculatedProjectileVelocity = requiredProjectileVelocity;
        calculatedPitch = midpointPitch;
      } else {
        calculatedProjectileVelocity = 0.0;
        calculatedPitch = midpointPitch;
      }
    }

    // Convert solved projectile velocity back into required flywheel surface
    // velocity
    double calculatedFlywheelVelocity = calculatedProjectileVelocity / ShooterSolverConstants.flywheelEfficiency;

    return new ShooterControls(
        RadiansPerSecond.of(
            calculatedFlywheelVelocity / ShooterConstants.flywheelRadius.in(Meters)),
        RadiansPerSecond.of(
            (calculatedFlywheelVelocity / ShooterConstants.flywheelRadius.in(Meters))
                * ShooterSolverConstants.lowerFlywheelVelocityMultiplier.get()),
        calculatedPitch,
        finalYaw,
        true);
  }

  /**
   * Checks if a calculated pitch is within the allowed shooter pitch range.
   *
   * @param pitch Pitch angle to validate.
   * @return True if the pitch is within the mechanical limits of the shooter.
   */
  private static boolean isValidPitch(Angle pitch) {
    return pitch.gte(ShooterConstants.minShooterPitch)
        && pitch.lte(ShooterConstants.maxShooterPitch);
  }

  public static ShooterControls getInterpolatedShooterControls(
      Pose2d odometryPose, ChassisSpeeds robotRelativeSpeeds) {

    Translation3d goal = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
        ? FieldConstants.redGoal
        : FieldConstants.blueGoal;

    // 1) Phase delay pose prediction
    double phaseDelaySec = ShooterSolverConstants.defaultShotLatencySec;
    Pose2d releasePose = odometryPose.exp(
        new Twist2d(
            robotRelativeSpeeds.vxMetersPerSecond * phaseDelaySec,
            robotRelativeSpeeds.vyMetersPerSecond * phaseDelaySec,
            robotRelativeSpeeds.omegaRadiansPerSecond * phaseDelaySec));

    // 2) Shooter position in field frame (robot center + shooter offset rotated
    // into field)
    Translation2d shooterOffsetField = ShooterConstants.shooterInitalPosition
        .getTranslation()
        .toTranslation2d()
        .rotateBy(releasePose.getRotation());

    Translation2d releaseShooterPosField = releasePose.getTranslation().plus(shooterOffsetField);
    Translation2d targetField = goal.toTranslation2d();

    // 3) Field-relative velocity (for lookahead)
    Translation2d chassisVelField = new Translation2d(
        robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond)
        .rotateBy(releasePose.getRotation());

    // include tangential velocity from omega. .
    double omega = robotRelativeSpeeds.omegaRadiansPerSecond;
    Translation2d tangentialVelField = new Translation2d(-omega * shooterOffsetField.getY(),
        omega * shooterOffsetField.getX());

    Translation2d shooterVelField = chassisVelField.plus(tangentialVelField);

    // 4) Lookahead iteration: TOF(distance) map -> shift shooter origin ->
    // recompute distance
    Translation2d lookaheadShooterPos = releaseShooterPosField;
    double dist = targetField.getDistance(lookaheadShooterPos);

    double tof = timeOfFlightMap.get(dist); // initial guess
    for (int i = 0; i < 15; i++) {
      tof = timeOfFlightMap.get(dist);
      Translation2d offset = shooterVelField.times(tof);
      lookaheadShooterPos = releaseShooterPosField.plus(offset);
      dist = targetField.getDistance(lookaheadShooterPos);
    }

    // 5) Drive yaw from lookahead shooter origin
    Translation2d lookaheadVector = targetField.minus(lookaheadShooterPos);
    Rotation2d driveYaw = lookaheadVector.getAngle();

    // 6) Interpolated setpoints from distance
    Rotation2d hoodAngle = hoodAngleMap.get(dist);
    double flywheelSurfaceSpeed = flywheelSpeedMap.get(dist);

    // 7) Convert flywheel surface speed -> angular velocity
    double flywheelRadPerSec = flywheelSurfaceSpeed / ShooterConstants.flywheelRadius.in(Meters);

    return new ShooterControls(
        RadiansPerSecond.of(flywheelRadPerSec / ShooterSolverConstants.flywheelEfficiency),
        RadiansPerSecond.of(
            (flywheelRadPerSec / ShooterSolverConstants.flywheelEfficiency)
                * ShooterSolverConstants.lowerFlywheelVelocityMultiplier.get()),
        Radians.of(hoodAngle.getRadians()),
        driveYaw,
        true);
  }
}
