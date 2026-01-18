package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final FlywheelIO flywheel;
  private final PivotIO pivot;
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();

  private final Alert flywheelDisconnectedAlert =
      new Alert("Flywheel disconnected", AlertType.kError);
  private final Alert pivotDisconnectedAlert = new Alert("Pivot disconnected", AlertType.kError);

  public Shooter(FlywheelIO flywheel, PivotIO pivot) {
    this.flywheel = flywheel;
    this.pivot = pivot;
  }

  @Override
  public void periodic() {
    flywheel.updateInputs(flywheelInputs);
    pivot.updateInputs(pivotInputs);

    Logger.processInputs("Shooter/Flywheel", flywheelInputs);
    Logger.processInputs("Shooter/Pivot", pivotInputs);

    flywheelDisconnectedAlert.set(
        !flywheelInputs.connected && Constants.currentMode != Constants.Mode.SIM);
    pivotDisconnectedAlert.set(
        !pivotInputs.connected && Constants.currentMode != Constants.Mode.SIM);
  }
}
