// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kitCollector;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Collector subsystem — brings in the collector I/O and logs inputs
 * Use this to run the intake motor and monitor its sensors.
 */
public class Collector extends SubsystemBase {

  private final CollectorIO io;
  private final CollectorIOInputsAutoLogged inputs = new CollectorIOInputsAutoLogged();

  private static final LoggedTunableNumber collectorKP =
      new LoggedTunableNumber("Collector/kP", CollectorConstants.kP);
  private static final LoggedTunableNumber collectorKD =
      new LoggedTunableNumber("Collector/kD", CollectorConstants.kD);
  private static final LoggedTunableNumber collectorKS =
      new LoggedTunableNumber("Collector/kS", CollectorConstants.kS);
  private static final LoggedTunableNumber collectorKV =
      new LoggedTunableNumber("Collector/kV", CollectorConstants.kV);
  private static final LoggedTunableNumber collectorKA =
      new LoggedTunableNumber("Collector/kA", CollectorConstants.kA);

  public Collector(CollectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // refresh inputs from hardware and feed to logger 
    io.updateInputs(inputs); 
    Logger.processInputs(getName(), inputs);

    if (collectorKP.hasChanged(hashCode())
        || collectorKD.hasChanged(hashCode())
        || collectorKS.hasChanged(hashCode())
        || collectorKV.hasChanged(hashCode())
        || collectorKA.hasChanged(hashCode())) {
      Elastic.sendNotification(
          new Notification(NotificationLevel.INFO, "Collector PID Gains Updated", "", 3000));
      io.setPID(
          new Slot0Configs()
              .withKP(collectorKP.get())
              .withKD(collectorKD.get())
              .withKS(collectorKS.get())
              .withKV(collectorKV.get())
              .withKA(collectorKA.get()));
    }
  }

  public void setVelocity(AngularVelocity velocity) {
    io.intake(velocity);
  }
}
