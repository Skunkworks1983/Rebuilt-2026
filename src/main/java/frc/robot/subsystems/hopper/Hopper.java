// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.Elastic;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/** {@summary} Requires one HopperIOTalonFX and can run said motor at a defined velocity */
public class Hopper extends SubsystemBase {
  HopperIO hopper;
  Alert hopperDisconnectAlert = new Alert(getName(), AlertType.kError);

  private final SysIdRoutine sysId;

  public final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  private static final LoggedTunableNumber hopperlKP =
      new LoggedTunableNumber("Hopper/kP", HopperConstants.hopperKP);
  private static final LoggedTunableNumber hopperlKD =
      new LoggedTunableNumber("Hopper/kD", HopperConstants.hopperKD);
  private static final LoggedTunableNumber hopperlKS =
      new LoggedTunableNumber("Hopper/kS", HopperConstants.hopperKS);
  private static final LoggedTunableNumber hopperlKV =
      new LoggedTunableNumber("Hopper/kV", HopperConstants.hopperKV);
  private static final LoggedTunableNumber hopperlKA =
      new LoggedTunableNumber("Hopper/kA", HopperConstants.hopperKA);

  public Hopper(HopperIO hopperMotor) {
    this.hopper = hopperMotor;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                HopperConstants.sysIdRampRate,
                HopperConstants.sysIdStepSize,
                null,
                (state) -> SignalLogger.writeString("Hopper/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runHopperTorqueCurrent(voltage.in(Volts)), null, this));
  }

  private void runHopperTorqueCurrent(double torqueCurrent) {
    hopper.setOpenLoopTorqueCurrent(Amps.of(torqueCurrent));
  }

  @Override
  public void periodic() {
    hopper.updateInputs(inputs);
    Logger.processInputs("Hopper/Hopper Motor", inputs);
    hopperDisconnectAlert.set(!inputs.connected);

    if (Constants.tuningMode) {

      int hash = hashCode();
      if (hopperlKP.hasChanged(hash)
          || hopperlKD.hasChanged(hash)
          || hopperlKS.hasChanged(hash)
          || hopperlKV.hasChanged(hash)
          || hopperlKA.hasChanged(hash)) {
        Slot0Configs gains = new Slot0Configs();
        gains.kP = hopperlKP.get();
        gains.kD = hopperlKD.get();
        gains.kS = hopperlKS.get();
        gains.kV = hopperlKV.get();
        gains.kA = hopperlKA.get();
        hopper.setPID(gains);

        Elastic.sendNotification(
            new Elastic.Notification(
                Elastic.NotificationLevel.INFO,
                "Hopper PID Update",
                "Hopper motor PID gains have been updated."));
      }
    }
  }

  /** sets the velocity of the hopper to the defined velocity */
  public void setHopperVelocity(AngularVelocity velocity) {
    hopper.setHopperVelocity(velocity);
  }

  public void setHopperDutyCycle(double dutyCycle) {
    hopper.setDutyCycle(dutyCycle);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runHopperTorqueCurrent(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runHopperTorqueCurrent(0.0))
        .withTimeout(1.0)
        .andThen(sysId.dynamic(direction));
  }
}
