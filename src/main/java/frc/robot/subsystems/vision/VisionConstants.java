// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.util.LoggedTunableNumber;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "limelight-a";
  public static String camera1Name = "camera_1";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, Math.PI));
  public static Transform3d robotToCamera1 =
      new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  /**
   * Simulation camera hardware properties Currently using Limelight 4 Properties
   *
   * @see <a
   *     href="https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-4">Limelight
   *     4 Hardware Specs (data source)</a>
   */
  public static class cameraSimulationParameters {
    public static final LoggedTunableNumber cameraResWidth =
        new LoggedTunableNumber("Vision/sim/Camera Width Px", 1280);
    public static final LoggedTunableNumber cameraResHeight =
        new LoggedTunableNumber("Vision/sim/Camera Height Px", 800);
    public static final LoggedTunableNumber cameraDiagonalFOVDegrees =
        new LoggedTunableNumber("Vision/sim/Camera Diagonal FOV Degrees", 96.4);
    public static final LoggedTunableNumber cameraFPS =
        new LoggedTunableNumber("Vision/sim/Camera FPS", 120);
    public static final LoggedTunableNumber cameraAverageLatencyMS =
        new LoggedTunableNumber("Vision/sim/Camera Average Latency MS", 30);
    public static final LoggedTunableNumber cameraLatencyStdDevMs =
        new LoggedTunableNumber("Vision/sim/Camera Latency StdDev MS", 5);
    public static final LoggedTunableNumber cameraAvgErrorPx =
        new LoggedTunableNumber("Vision/sim/Camera Average Error PX", 0.25);
    public static final LoggedTunableNumber cameraErrorStdDevPx =
        new LoggedTunableNumber("Vision/sim/Camera Error StdDev PX", 0.08);
  }
}
