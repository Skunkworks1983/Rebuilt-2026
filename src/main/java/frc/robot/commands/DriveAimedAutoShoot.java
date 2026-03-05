package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveAimedShoot.TargetType;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.AllianceZone;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveAimedAutoShoot extends Command {
  private static final String logKey = "Commands/Auto Shoot/Drive Aimed Auto";

  private final Drive drive;
  private final Shooter shooter;
  private final DriveAimedShoot goalCommand;
  private final DriveAimedShoot transferCommand;

  private final Trigger allianceZoneTrigger;

  private DriveAimedShoot activeCommand;

  public DriveAimedAutoShoot(
      Drive drive,
      Shooter shooter,
      Command indexerCommand,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    this.drive = drive;
    this.shooter = shooter;
    goalCommand =
        new DriveAimedShoot(drive, shooter, indexerCommand, xSupplier, ySupplier, TargetType.GOAL);
    transferCommand =
        new DriveAimedShoot(
            drive, shooter, indexerCommand, xSupplier, ySupplier, TargetType.TRANSFER);

    allianceZoneTrigger = new Trigger(this::isInAllianceZone);
    allianceZoneTrigger.onTrue(Commands.runOnce(() -> switchTo(goalCommand)));
    allianceZoneTrigger.onFalse(Commands.runOnce(() -> switchTo(transferCommand)));
  }

  @Override
  public void initialize() {
    boolean inAllianceZone = isInAllianceZone();
    Logger.recordOutput(logKey, true);
    switchTo(inAllianceZone ? goalCommand : transferCommand);
  }

  @Override
  public void end(boolean interrupted) {
    if (activeCommand != null) {
      CommandScheduler.getInstance().cancel(activeCommand);
      activeCommand = null;
    }
    CommandScheduler.getInstance().cancel(goalCommand);
    CommandScheduler.getInstance().cancel(transferCommand);

    shooter.stopFlywheels();
    shooter.setPivotHome();
    Logger.recordOutput(logKey, false);
    Logger.recordOutput(logKey + "/ActiveTarget", "NONE");
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private boolean isInAllianceZone() {
    Translation2d robotPosition = drive.getPose().getTranslation();
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    AllianceZone zone =
        alliance == Alliance.Red ? FieldConstants.redAllianceZone : FieldConstants.blueAllianceZone;
    return zone.contains(robotPosition);
  }

  private void switchTo(DriveAimedShoot nextCommand) {
    if (activeCommand == nextCommand) {
      return;
    }
    if (activeCommand != null) {
      CommandScheduler.getInstance().cancel(activeCommand);
    }
    activeCommand = nextCommand;
    CommandScheduler.getInstance().schedule(activeCommand);
    Logger.recordOutput(logKey + "/ActiveTarget", nextCommand == goalCommand ? "GOAL" : "TRANSFER");
  }
}
