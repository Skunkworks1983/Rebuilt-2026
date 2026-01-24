package frc.robot.subsystems.collector;

import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class CollectorModuleSimIO implements CollectorModuleIO {
  private IntakeSimulation collectorSimulation;

  public CollectorModuleSimIO(AbstractDriveTrainSimulation driveTrain) {

    this.collectorSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            driveTrain,
            Meters.of(0.7),
            Meters.of(0.2),
            IntakeSimulation.IntakeSide.BACK,
            1);
  }

  @Override
  public void setIntake(boolean runIntake) {
    if (runIntake) collectorSimulation.startIntake();
    else collectorSimulation.stopIntake();
  }

  @Override
  public boolean isFuelInsideCollector() {
    return collectorSimulation.getGamePiecesAmount() != 0;
  }

  @Override
  public void updateInputs(CollectorIOInputs input) {
    // input.extended = collectorSimulation
    // input.speed = collectorSimulation
    // input.amountOfFuelInside = collectorSimulation.getGamePiecesAmount();
  }

  // @Override
  // public void launchNote() {
  //     if (collectorSimulation.obtainGamePieceFromIntake())
  //         ShooterIOSim.launchNote();
  // }
}
