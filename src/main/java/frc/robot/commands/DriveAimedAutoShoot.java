package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveAimedShoot.TargetType;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.FieldConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveAimedAutoShoot extends Command {
  private static final String logKey = "Commands/Auto Shoot/Drive Aimed Auto";

  private final Drive drive;
  private final Shooter shooter;
  private final DriveAimedShoot goalCommand;
  private final DriveAimedShoot transferNeutralCommand;

  private final DriveAimedShoot transferFarCommand;

  private DriveAimedShoot activeCommand;

  private enum ZoneType {
    ALLIANCE,
    NEUTRAL,
    FAR
  }

  private ZoneType lastZone;

  public DriveAimedAutoShoot(
      Drive drive,
      Shooter shooter,
      Vision vision,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      BooleanSupplier indexPermitted,
      Command feedCommand) {
    this.drive = drive;
    this.shooter = shooter;
    goalCommand =
        new DriveAimedShoot(
            drive,
            shooter,
            vision,
            xSupplier,
            ySupplier,
            TargetType.GOAL,
            indexPermitted,
            feedCommand);
    transferNeutralCommand =
        new DriveAimedShoot(
            drive,
            shooter,
            vision,
            xSupplier,
            ySupplier,
            TargetType.TRANSFER_NEUTRAL,
            indexPermitted,
            feedCommand);

    transferFarCommand =
        new DriveAimedShoot(
            drive,
            shooter,
            vision,
            xSupplier,
            ySupplier,
            TargetType.TRANSFER_FAR,
            indexPermitted,
            feedCommand);
  }

  @Override
  public void initialize() {
    ZoneType zone = getCurrentZone();
    lastZone = zone;
    Logger.recordOutput(logKey, true);
    Logger.recordOutput(logKey + "/Zone", zone.name());
    switchTo(selectCommandForZone(zone));
  }

  @Override
  public void execute() {
    ZoneType zone = getCurrentZone();
    Logger.recordOutput(logKey + "/Zone", zone.name());
    Logger.recordOutput(logKey + "/InAllianceZone", zone == ZoneType.ALLIANCE);

    if (zone != lastZone) {
      switchTo(selectCommandForZone(zone));
      lastZone = zone;
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (activeCommand != null) {
      CommandScheduler.getInstance().cancel(activeCommand);
      activeCommand = null;
    }
    CommandScheduler.getInstance().cancel(goalCommand);
    CommandScheduler.getInstance().cancel(transferNeutralCommand);
    CommandScheduler.getInstance().cancel(transferFarCommand);

    shooter.stopFlywheels();
    shooter.setPivotHome();
    Logger.recordOutput(logKey, false);
    Logger.recordOutput(logKey + "/InAllianceZone", false);
    Logger.recordOutput(logKey + "/Zone", "NONE");
    Logger.recordOutput(logKey + "/ActiveTarget", "NONE");
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private ZoneType getCurrentZone() {
    Translation2d robotPosition = drive.getPose().getTranslation();
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    boolean inBlue = FieldConstants.blueAllianceZone.contains(robotPosition);
    boolean inRed = FieldConstants.redAllianceZone.contains(robotPosition);

    if (alliance == Alliance.Blue) {
      if (inBlue) {
        return ZoneType.ALLIANCE;
      }
      if (inRed) {
        return ZoneType.FAR;
      }
    } else {
      if (inRed) {
        return ZoneType.ALLIANCE;
      }
      if (inBlue) {
        return ZoneType.FAR;
      }
    }

    return ZoneType.NEUTRAL;
  }

  private DriveAimedShoot selectCommandForZone(ZoneType zone) {
    return switch (zone) {
      case ALLIANCE -> goalCommand;
      case NEUTRAL -> transferNeutralCommand;
      case FAR -> transferFarCommand;
    };
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
    String targetLabel;
    if (nextCommand == goalCommand) {
      targetLabel = "GOAL";
    } else if (nextCommand == transferNeutralCommand) {
      targetLabel = "TRANSFER_NEUTRAL";
    } else {
      targetLabel = "TRANSFER_FAR";
    }
    Logger.recordOutput(logKey + "/ActiveTarget", targetLabel);
  }

  /** Forces auto aimed yaw setpoint and shooter setpoints to remain fixed */
  public void lockControls() {
    if (activeCommand != null) {
      activeCommand.lockControls();
    }
  }

  /** Allows auto aimed shoot yaw and shooter setpoint to change */
  public void unlockControls() {
    if (activeCommand != null) {
      activeCommand.unlockControls();
    }
  }
}
