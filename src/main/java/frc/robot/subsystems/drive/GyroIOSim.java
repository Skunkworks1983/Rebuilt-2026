package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.util.Units;
import frc.robot.util.PhoenixUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;

/**
 * Physics sim implementation of {@link GyroIO}.
 *
 * <p>Uses a maple sim managed {@link GyroSimulation} to provide gyro readings. Timestamps are
 * calculated using Phoenix CAN chain simulation.
 *
 * @see <a
 *     href="https://shenzhen-robotics-alliance.github.io/maple-sim/swerve-sim-hardware-abstraction/#1-interacting-with-the-gyro-through-io-abstraction">MapleSim
 *     Simulated Gyro Docs</a>
 * @author Eddy W
 */
public class GyroIOSim implements GyroIO {
  private final GyroSimulation gyroSimulation;

  public GyroIOSim(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = gyroSimulation.getGyroReading();
    inputs.yawVelocityRadPerSec =
        Units.degreesToRadians(gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond));

    inputs.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
    inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
  }
}
