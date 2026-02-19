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
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
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
      AngularVelocity shooterVelocity,
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
   * Calculates shooter controls for aiming at the goal using linear
   * interpolation. Compensates for
   * robot velocity to enable shooting while moving.
   *
   * @param speeds                 Current robot chassis speeds
   * @param odometryPose           Current robot pose on the field.
   * @param shooterInterporlations Lookup table helper for goal shots.
   * @return Calculated {@link ShooterControls}.
   */
  public static ShooterControls getShooterControlsGoal(
      ChassisSpeeds speeds, Pose2d odometryPose, ShooterInterpolation shooterInterporlations) {
    Translation3d target = DriverStation.getAlliance().get() == Alliance.Red
        ? FieldConstants.redGoal
        : FieldConstants.blueGoal;

    Translation2d targetVector = target.toTranslation2d().minus(odometryPose.getTranslation());

    Distance distance = Meters.of(targetVector.getNorm());

    AngularVelocity shooterVelocity = shooterInterporlations.getShooterVelocityGoal(distance);
    Angle shooterPitch = shooterInterporlations.getPivotAngleGoal(distance);
    Rotation2d rawYaw = targetVector.getAngle();

    // Get field relative chassis speeds
    Translation2d robotVel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
        .rotateBy(odometryPose.getRotation());
    double fieldVx = robotVel.getX();
    double fieldVy = robotVel.getY();

    // Unit vectors of the shot direction (field frame)
    double ux = rawYaw.getCos();
    double uy = rawYaw.getSin();

    // Components of the robot velocity along the shot-forward and shot-lateral
    // directions
    double parallelComponent = fieldVx * ux + fieldVy * uy; // Velocity along direction of shot
    double perpendicularComponent = fieldVx * -uy + fieldVy * ux; // Velocity perpendicular to shot

    // Flywheel parallel velocity
    double shooterLinear = shooterVelocity.in(RadiansPerSecond)
        * ShooterConstants.flywheelRadius.in(Meters)
        * Math.cos(shooterPitch.in(Radians));

    Angle compensationAngle = Radians.of(Math.atan2(perpendicularComponent, shooterLinear + parallelComponent));

    return new ShooterControls(
        shooterVelocity, shooterPitch, rawYaw.minus(new Rotation2d(compensationAngle)), true);
  }

  /**
   * Runs a convergence test for the numerical solver across the field to verify
   * stability. Prints
   * results to stdout.
   *
   * @param stepMeters            Grid step size for the test in meters.
   * @param prevPitchDegrees      Array of initial pitch guesses to test.
   * @param prevFlywheelRotPerSec Array of initial flywheel velocity guesses to
   *                              test.
   */
  public static void runOptimizedSolverConvergenceTest(
      double stepMeters, double[] prevPitchDegrees, double[] prevFlywheelRotPerSec) {

    int totalCases = 0;
    int convergedCount = 0;
    double maxAbsError = 1;
    String worstCase = "";

    double minX = 0;
    double minY = 0;

    double maxX = Units.inchesToMeters(156.11);
    double maxY = Units.inchesToMeters(317.69);

    ChassisSpeeds zeroSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    for (double x = minX; x <= maxX; x += stepMeters) {
      for (double y = minY; y <= maxY; y += stepMeters) {
        Pose2d pose = new Pose2d(x, y, new Rotation2d());
        double distanceToTarget = pose.getTranslation().getDistance(FieldConstants.blueGoal.toTranslation2d());
        for (double initialPitch : prevPitchDegrees) {
          for (double initialFlywheel : prevFlywheelRotPerSec) {
            totalCases++;

            // Get calculated controls
            ShooterControls out = getNumericalSolvedShooterControls(
                pose,
                zeroSpeeds,
                Degrees.of(initialPitch),
                RotationsPerSecond.of(initialFlywheel));

            // Recompute error
            double deltaZ = FieldConstants.blueGoal.getZ() - ShooterConstants.shooterInitalPosition.getZ();

            double pitch = out.shooterPitch().in(Radians);
            double flywheelLinear = out.shooterVelocity().in(RadiansPerSecond)
                * ShooterConstants.flywheelRadius.in(Meters);

            double cosTheta = Math.cos(pitch);
            double sinTheta = Math.sin(pitch);

            double effectiveVelocity = flywheelLinear * cosTheta;

            double term1 = (flywheelLinear * sinTheta * distanceToTarget) / effectiveVelocity;
            double term2 = (ShooterSolverConstants.gravity * distanceToTarget * distanceToTarget)
                / (2 * effectiveVelocity * effectiveVelocity);

            double e = term1 - term2 - deltaZ;
            double absErr = Math.abs(e);
            if (absErr <= ShooterSolverConstants.maxError)
              convergedCount++;

            if (absErr > maxAbsError && out.possibleShot() == true) {
              maxAbsError = absErr;
              worstCase = String.format(
                  "pose=(%.2f,%.2f) initPitch=%.3f initFlywheel=%.2f -> finalPitch=%.3f finalFlywheelLin=%.3f absErr=%.6f",
                  x, y, initialPitch, initialFlywheel, pitch, flywheelLinear, absErr);
            }
          }
        }
      }
    }

    System.out.println("=== Optimized Solver Convergence Test Summary ===");
    System.out.println("Total cases: " + totalCases);
    System.out.println("Converged (|residual| <= maxError): " + convergedCount);
    System.out.println(
        "Convergence rate: " + (100.0 * convergedCount / Math.max(1, totalCases)) + "%");
    System.out.println("Max absolute residual encountered: " + maxAbsError + " meters");
    System.out.println("Worst case: " + worstCase);
    System.out.println("Max error threshold used: " + ShooterSolverConstants.maxError);
  }

  /**
   * Calculates shooter controls using an apex-constrained ballistic model.
   * Assumes the shot peaks
   * at a specific height clearance above the goal.
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
            robotRelativeSpeeds.vxMetersPerSecond * ShooterSolverConstants.defaultShotLatencySec,
            robotRelativeSpeeds.vyMetersPerSecond * ShooterSolverConstants.defaultShotLatencySec,
            robotRelativeSpeeds.omegaRadiansPerSecond * ShooterSolverConstants.defaultShotLatencySec));
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

    // Apex height calculation
    double heightTrajectoryPeak = relativeGoalZ + ShooterSolverConstants.hubHeightClearance.in(Meters);
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
    Translation2d stationaryLaunchVectorField = new Translation2d(requiredHorizontalSpeed, angleToGoal);

    // Average field velocity during flight
    Translation2d chassisVelField = new Translation2d(
        robotRelativeSpeeds.vxMetersPerSecond,
        robotRelativeSpeeds.vyMetersPerSecond)
        .rotateBy(releasePose.getRotation());

    double omega = robotRelativeSpeeds.omegaRadiansPerSecond;

    Translation2d tangentialVelField = new Translation2d(
        -omega * shooterOffsetField.getY(),
        omega * shooterOffsetField.getX());

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

    Translation3d virtualGoal = new Translation3d(
        goal.getX() - leadShiftField.getX(), goal.getY() - leadShiftField.getY(), goal.getZ());

    Translation2d virtualTargetVector = virtualGoal.toTranslation2d().minus(releaseShooterPosField);

    Logger.recordOutput("Shooter/Goal/Real", goal);
    Logger.recordOutput("Shooter/Goal/Virtual", virtualGoal);
    Logger.recordOutput("Shooter/Goal/LeadShiftXY", leadShiftField);
    Logger.recordOutput("Shooter/TargetVector/Real", targetVector);
    Logger.recordOutput("Shooter/TargetVector/Virtual", virtualTargetVector);

    // Should be ~0
    Translation2d shouldMatchStationary = effectiveLaunchVectorField.plus(robotLinearVelocityField);
    Logger.recordOutput("Shooter/StationaryMinusReconstructedError",
        shouldMatchStationary.minus(stationaryLaunchVectorField).getNorm());

    return new ShooterControls(
        RadiansPerSecond.of(flywheelSurfaceSpeed / ShooterConstants.flywheelRadius.in(Meters)),
        Radians.of(finalPitch),
        finalDriveYaw,
        true);
  }

  /**
   * Calculates shooter controls using an iterative numerical solver (Gradient
   * Descent). Solves for
   * pitch and velocity simultaneously to minimize error.
   *
   * @param odometryPose         Current robot pose.
   * @param chassisSpeeds        Current robot chassis speeds.
   * @param prevShooterPitch     Previous pitch setpoint for seeding the solver.
   * @param prevFlywheelVelocity Previous flywheel velocity setpoint for seeding
   *                             the solver.
   * @return Calculated {@link ShooterControls}.
   */
  public static ShooterControls getNumericalSolvedShooterControls(
      Pose2d odometryPose,
      ChassisSpeeds chassisSpeeds,
      Angle prevShooterPitch,
      AngularVelocity prevFlywheelVelocity) {
    // Translation3d target = DriverStation.getAlliance().get() == Alliance.Red
    // ? FieldConstants.redGoal
    // : FieldConstants.blueGoal;

    Translation3d target = FieldConstants.blueGoal;

    Translation2d targetVector = target.toTranslation2d().minus(odometryPose.getTranslation());

    // Intentionally move everything to doubles for math in iterative steps
    double x = targetVector.getNorm();
    double z = target.getZ() - ShooterConstants.shooterInitalPosition.getZ();

    Rotation2d rawYaw = targetVector.getAngle();
    double shotAngleRadians = rawYaw.getRadians();

    double velocityRobotAlongShot = chassisSpeeds.vxMetersPerSecond * Math.cos(shotAngleRadians)
        + chassisSpeeds.vyMetersPerSecond * Math.sin(shotAngleRadians);

    double shooterPitch = prevShooterPitch.in(Radians);
    double flywheelLinearVelocity = prevFlywheelVelocity.in(RadiansPerSecond)
        * ShooterConstants.flywheelRadius.in(Meters);

    if (x < ShooterSolverConstants.minShootDistanceHub.in(Meters)) {
      return new ShooterControls(prevFlywheelVelocity, prevShooterPitch, rawYaw, false);
    }

    for (int i = 0; i < ShooterSolverConstants.maxIterations; i++) {
      // Trig once for optimization
      double cosTheta = Math.cos(shooterPitch);
      double sinTheta = Math.sin(shooterPitch);

      // Velocity of flywheel + robot velocity in flywheel direction
      double effectiveVelocity = flywheelLinearVelocity * cosTheta + velocityRobotAlongShot;

      // Error of current pitch and velocity from target
      double e = ((flywheelLinearVelocity * sinTheta * x) / effectiveVelocity)
          - (ShooterSolverConstants.gravity * x * x)
              / (2 * effectiveVelocity * effectiveVelocity)
          - z;

      if (Math.abs(e) < ShooterSolverConstants.maxError)
        break;

      double effectiveVelocitySq = effectiveVelocity * effectiveVelocity;
      double effectiveVelocityCu = effectiveVelocitySq * effectiveVelocity;

      // Partial derivative Pitch (theta)
      double slopeTerm_dTheta = (x
          * flywheelLinearVelocity
          * (flywheelLinearVelocity + velocityRobotAlongShot * cosTheta))
          / effectiveVelocitySq;

      double gravityTerm_dTheta = (ShooterSolverConstants.gravity * x * x * (-flywheelLinearVelocity * sinTheta))
          / effectiveVelocityCu;

      double de_dtheta = slopeTerm_dTheta + gravityTerm_dTheta;

      // Partial derivative Velocity
      double slopeTerm_dV = (x * velocityRobotAlongShot * sinTheta) / effectiveVelocitySq;

      double gravityTerm_dV = (ShooterSolverConstants.gravity * x * x * cosTheta) / effectiveVelocityCu;

      double de_dv = slopeTerm_dV + gravityTerm_dV;

      double dThetaWeighted = de_dtheta * ShooterSolverConstants.weightTheta;
      double dVelocityWeighted = de_dv * ShooterSolverConstants.weightVelocity;

      // Update step using Gradient Descent
      double denominator = (dThetaWeighted * dThetaWeighted + dVelocityWeighted * dVelocityWeighted);

      // Safety epsilon to prevent NaN
      double lambda = e / (denominator + 1e-6);

      shooterPitch -= lambda * dThetaWeighted * ShooterSolverConstants.changeRate;
      flywheelLinearVelocity -= lambda * dVelocityWeighted * ShooterSolverConstants.changeRate;
    }

    // Get field relative chassis speeds
    Translation2d fieldRelativeRobotVel = new Translation2d(chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond)
        .rotateBy(odometryPose.getRotation());

    // Unit vectors of the shot direction (do once to minimize trig)
    double ux = rawYaw.getCos();
    double uy = rawYaw.getSin();

    double parallelComponent = fieldRelativeRobotVel.getX() * ux + fieldRelativeRobotVel.getY() * uy;

    double perpendicularComponent = fieldRelativeRobotVel.getX() * -uy + fieldRelativeRobotVel.getY() * ux;

    Angle compensationAngle = Radians
        .of(Math.atan2(perpendicularComponent, flywheelLinearVelocity + parallelComponent));

    double effectiveVelocity = flywheelLinearVelocity * Math.cos(shooterPitch) + parallelComponent;

    double term1 = (flywheelLinearVelocity * Math.sin(shooterPitch) * x) / effectiveVelocity;
    double term2 = (0.5 * ShooterSolverConstants.gravity * x * x) / (effectiveVelocity * effectiveVelocity);

    double e = term1 - term2 - z;

    if (e > ShooterSolverConstants.maxError) {
      return new ShooterControls(
          prevFlywheelVelocity,
          prevShooterPitch,
          rawYaw.minus(new Rotation2d(compensationAngle)),
          true);
    }

    return new ShooterControls(
        RadiansPerSecond.of(flywheelLinearVelocity / ShooterConstants.flywheelRadius.in(Meters)),
        Radians.of(shooterPitch),
        rawYaw.minus(new Rotation2d(compensationAngle)),
        true);
  }

  /**
   * Calculates shooter controls for the "transfer" shot (shooting into the
   * alliance zone from the
   * transfer zone). Includes logic to avoid hitting the hub structure.
   *
   * <p>
   * It is attempted to first calculate shooter pitch from the current velocity
   * (beause we can
   * change pitch faster than velocity and it uses less amperage) If this fails,
   * we set pitch to be
   * the midpoint of the possible pitch range, and caluclate needed flywheel
   * velocity (this ensures
   * that we minimize having to change flywheel velocity)
   *
   * @param speeds                 Current robot chassis speeds.
   * @param odometryPose           Current robot pose.
   * @param flywheelLinearVelocity Current flywheel linear velocity
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

    // PART 1: YAW/ACTUAL TARGET CALCULATER (Hub avoidance)
    Translation2d[] corners = new Translation2d[] {
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

      Rotation2d angleToCorner = robotToCorner.getAngle().minus(rawYaw); // Using Rotation2d to handle angle wrapping

      Angle angleToCornerMeasure = Radians.of(angleToCorner.getRadians());

      if (angleToCornerMeasure.gt(hubFurthestCCWAngle))
        hubFurthestCCWAngle = angleToCornerMeasure;
      if (angleToCornerMeasure.lt(hubFurthestCWAngle))
        hubFurthestCWAngle = angleToCornerMeasure;
    }

    // 5. Collision Detection & Avoidance Logic
    boolean hubIsCloser = minDistToHub.lt(distanceToTarget);
    boolean lineOfSightBlocked = hubFurthestCWAngle.lt(Radians.of(0)) && hubFurthestCCWAngle.gt(Radians.of(0));

    if (hubIsCloser && lineOfSightBlocked) {
      Angle rightCandidate = hubFurthestCWAngle.minus(ShooterSolverConstants.transferShotSafetyMargin);
      Angle leftCandidate = hubFurthestCCWAngle.plus(ShooterSolverConstants.transferShotSafetyMargin);

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
    double flywheelLinearVelocitySquared = currentFlywheelLienarVelocity * currentFlywheelLienarVelocity;
    double sinArgument = (ShooterSolverConstants.gravity * distanceToTargetMeters) / flywheelLinearVelocitySquared;

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
        double requiredV = Math.sqrt((ShooterSolverConstants.gravity * distanceToTargetMeters) / sinTwoTheta);
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
    Translation2d chassisVelField = new Translation2d(robotRelativeSpeeds.vxMetersPerSecond,
        robotRelativeSpeeds.vyMetersPerSecond)
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
        RadiansPerSecond.of(flywheelRadPerSec),
        Radians.of(hoodAngle.getRadians()),
        driveYaw,
        true);
  }

}
