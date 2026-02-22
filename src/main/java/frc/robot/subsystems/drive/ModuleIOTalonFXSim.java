package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import frc.robot.util.PhoenixUtil;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

/**
 * Physics sim implementation of module IO. The sim models are configured from an adjusted version
 * of phoenix tuner generated swerve constants. Simulation is always based on voltage control.
 * Phoenix simulated motor controllers are used for higher accuracy
 *
 * @see <a
 *     href="https://shenzhen-robotics-alliance.github.io/maple-sim/swerve-sim-hardware-abstraction/#2-interacting-with-the-simulated-modules-through-io-abstraction">MapleSim
 *     Swerve Sim Info</a>
 * @see <a
 *     href="https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/simulation/simulation-intro.html#simulation-api">CTRE
 *     Phoenix Motor Simulation API Info</a>
 * @author Eddy W
 */
public class ModuleIOTalonFXSim extends ModuleIOTalonFX {
  private final SwerveModuleSimulation simulation;

  public ModuleIOTalonFXSim(SwerveModuleConstants constants, SwerveModuleSimulation simulation) {
    super(PhoenixUtil.regulateModuleConstantForSimulation(constants));

    this.simulation = simulation;
    simulation.useDriveMotorController(new PhoenixUtil.TalonFXMotorControllerSim(driveTalon));

    simulation.useSteerMotorController(
        new PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(turnTalon, cancoder));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    super.updateInputs(inputs);

    // Update odometry inputs
    inputs.odometryTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();

    inputs.odometryDrivePositionsRad =
        Arrays.stream(simulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();

    inputs.odometryTurnPositions = simulation.getCachedSteerAbsolutePositions();
  }
}
