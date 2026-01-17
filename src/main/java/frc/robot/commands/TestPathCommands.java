package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class TestPathCommands {
        private static Command generateTestCommand(
                        Distance distance,
                        Rotation2d direction,
                        Rotation2d targetEndAngle,
                        boolean isRobotRelative,
                        LinearVelocity maxVelocity,
                        LinearAcceleration maxAcceleration,
                        AngularVelocity maxAngularVelocity,
                        AngularAcceleration maxAngularAcceleration,
                        Drive drive) {
                if (DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                                && !isRobotRelative) {
                        // Flip direction for red alliance so 0 degrees is always forward (only applies to field relative)
                        direction = direction.plus(new Rotation2d(Degrees.of(180)));
                }
                Translation2d initialLoc = drive.getPose().getTranslation();

                Translation2d offset = new Translation2d(distance.in(Meters), direction);

                List<Waypoint> waypoints;
                if (isRobotRelative) {
                        Translation2d finalLoc = initialLoc.plus(offset.rotateBy(
                                        drive.getPose().getRotation().plus(new Rotation2d(Degrees.of(180)))));
                        Logger.recordOutput("TestPaths/TargetPose", new Pose2d(finalLoc, targetEndAngle));

                        waypoints = PathPlannerPath.waypointsFromPoses(
                                        new Pose2d(initialLoc, offset.getAngle().plus(drive.getRotation())),
                                        new Pose2d(finalLoc, offset.getAngle().plus(drive.getRotation())));
                } else {
                        Translation2d finalLoc = initialLoc.plus(offset);
                        Logger.recordOutput("TestPaths/TargetPose", new Pose2d(finalLoc, targetEndAngle));

                        waypoints = PathPlannerPath.waypointsFromPoses(
                                        new Pose2d(initialLoc, offset.getAngle()),
                                        new Pose2d(finalLoc, offset.getAngle()));

                }
                PathConstraints constraints = new PathConstraints(
                                maxVelocity, maxAcceleration, maxAngularVelocity, maxAngularAcceleration);

                PathPlannerPath path = new PathPlannerPath(waypoints, constraints,
                                new IdealStartingState(0, drive.getRotation()),
                                new GoalEndState(0, targetEndAngle));
                // Since the translations of this path is generated directly from the robot's position, don' tflip path
                path.preventFlipping = true;
                Logger.recordOutput("TestPaths/PathHeading", offset.getAngle().plus(drive.getRotation()).getDegrees());
                Logger.recordOutput("TestPaths/Generated Path", Pose2d.struct,
                                path.getPathPoses().toArray(new Pose2d[0]));
                return AutoBuilder.followPath(path);
        }

        public static Command getTestPath(
                        Supplier<LinearVelocity> maxVelocity,
                        Supplier<Distance> distance,
                        Supplier<Rotation2d> direction,
                        BooleanSupplier useTargetEndAngle,
                        Supplier<Rotation2d> targetEndAngle,
                        BooleanSupplier isRobotRelative,
                        Drive drive) {
                return Commands.defer(
                                () -> generateTestCommand(
                                                distance.get(),
                                                direction.get(),
                                                useTargetEndAngle.getAsBoolean() ? targetEndAngle.get()
                                                                : drive.getRotation(),
                                                isRobotRelative.getAsBoolean(),
                                                maxVelocity.get(),
                                                MetersPerSecondPerSecond.of(
                                                                TestPathCommandConstants.maxAccelerationMPSPS.get()),
                                                DegreesPerSecond.of(
                                                                TestPathCommandConstants.maxAngularVelocityDPS.get()),
                                                DegreesPerSecondPerSecond.of(
                                                                TestPathCommandConstants.maxAngularAccelerationDPSPS
                                                                                .get()),
                                                drive),
                                Set.of(drive));
        }

        public static void publishTestPaths(Drive drive) {
                LoggedNetworkBoolean isRobotOriented = new LoggedNetworkBoolean("TestPaths/Is Robot Oriented?", false);
                LoggedNetworkBoolean useTargetEndAngle = new LoggedNetworkBoolean("TestPaths/Use target end angle?",
                                false);

                LoggedNetworkNumber maxVelocity = new LoggedNetworkNumber(
                                "TestPaths/Max Velocity Meters Per Sec", TestPathCommandConstants.maxVelocityMPS.get());

                LoggedNetworkNumber distance = new LoggedNetworkNumber("TestPaths/Distance Meters", 2.0);
                LoggedNetworkNumber direction = new LoggedNetworkNumber("TestPaths/Direction Degrees", 0.0);
                LoggedNetworkNumber targetEndAngle = new LoggedNetworkNumber("TestPaths/Target End Angle Degrees", 0.0);

                SmartDashboard.putData(
                                "TestPaths/Follow Path (Distance and Angle)",
                                getTestPath(
                                                () -> MetersPerSecond.of(maxVelocity.get()),
                                                () -> Meters.of(distance.get()),
                                                () -> new Rotation2d(Degrees.of(direction.get())),
                                                () -> useTargetEndAngle.get(),
                                                () -> new Rotation2d(Degrees.of(targetEndAngle.get())),
                                                () -> isRobotOriented.get(),
                                                drive));
                SmartDashboard.putData(
                                "TestPaths/Move Forward Distance",
                                getTestPath(
                                                () -> MetersPerSecond.of(maxVelocity.get()),
                                                () -> Meters.of(distance.get()),
                                                () -> new Rotation2d(Degrees.of(0)),
                                                () -> useTargetEndAngle.get(),
                                                () -> new Rotation2d(Degrees.of(targetEndAngle.get())),
                                                () -> isRobotOriented.get(),
                                                drive));
                SmartDashboard.putData(
                                "TestPaths/Move Back Distance",
                                getTestPath(
                                                () -> MetersPerSecond.of(maxVelocity.get()),
                                                () -> Meters.of(distance.get()),
                                                () -> new Rotation2d(Degrees.of(180)),
                                                () -> useTargetEndAngle.get(),
                                                () -> new Rotation2d(Degrees.of(targetEndAngle.get())),
                                                () -> isRobotOriented.get(),
                                                drive));
                SmartDashboard.putData(
                                "TestPaths/Move Left Distance",
                                getTestPath(
                                                () -> MetersPerSecond.of(maxVelocity.get()),
                                                () -> Meters.of(distance.get()),
                                                () -> new Rotation2d(Degrees.of(90)),
                                                () -> useTargetEndAngle.get(),
                                                () -> new Rotation2d(Degrees.of(targetEndAngle.get())),
                                                () -> isRobotOriented.get(),
                                                drive));
                SmartDashboard.putData(
                                "TestPaths/Move Right Distance",
                                getTestPath(
                                                () -> MetersPerSecond.of(maxVelocity.get()),
                                                () -> Meters.of(distance.get()),
                                                () -> new Rotation2d(Degrees.of(270)),
                                                () -> useTargetEndAngle.get(),
                                                () -> new Rotation2d(Degrees.of(targetEndAngle.get())),
                                                () -> isRobotOriented.get(),
                                                drive));
        }
}
