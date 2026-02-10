// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kitIndexer;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {

  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  private static final LoggedTunableNumber indexerKP =
      new LoggedTunableNumber("Indexer/kP", IndexerConstants.kP);
  private static final LoggedTunableNumber indexerKD =
      new LoggedTunableNumber("Indexer/kD", IndexerConstants.kD);
  private static final LoggedTunableNumber indexerKS =
      new LoggedTunableNumber("Indexer/kS", IndexerConstants.kS);
  private static final LoggedTunableNumber indexerKV =
      new LoggedTunableNumber("Indexer/kV", IndexerConstants.kV);
  private static final LoggedTunableNumber indexerKA =
      new LoggedTunableNumber("Indexer/kA", IndexerConstants.kA);

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    if (indexerKP.hasChanged(hashCode())
        || indexerKD.hasChanged(hashCode())
        || indexerKS.hasChanged(hashCode())
        || indexerKV.hasChanged(hashCode())
        || indexerKA.hasChanged(hashCode())) {
      Elastic.sendNotification(
          new Notification(NotificationLevel.INFO, "Indexer PID Gains Updated", "", 3000));
      io.setPID(
          new Slot0Configs()
              .withKP(indexerKP.get())
              .withKD(indexerKD.get())
              .withKS(indexerKS.get())
              .withKV(indexerKV.get())
              .withKA(indexerKA.get()));
    }
  }

  public void setSpeed(AngularVelocity velocity) {
    io.intake(velocity);
  }
}
