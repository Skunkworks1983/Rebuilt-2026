package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.util.List;

public class AutoPreview {
  public static void initAutoPreview(SendableChooser<String> autoName, Field2d field) {
    List<String> options = AutoBuilder.getAllAutoNames();
    System.out.println("Available autos for preview: " + options);
    autoName.onChange(
        (newAuto) -> {
          if (options.contains(newAuto)) {
            System.out.println("Visualizing auto: " + newAuto);
            visualizeAuto(newAuto, field);
          } else {
            clearField(field);
          }
        });
  }

  private static void visualizeAuto(String autoName, Field2d field) {
    List<PathPlannerPath> auto = null;
    try {
      auto = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
    } catch (Exception e) {
      DriverStation.reportError("Auto file for visualization not found", e.getStackTrace());
    }
    if (auto != null) {
      FieldObject2d fieldObj = field.getObject("Auto Preview");
      List<Pose2d> allPoses = auto.stream().flatMap(path -> path.getPathPoses().stream()).toList();
      fieldObj.setPoses(allPoses);
    }
  }

  private static void clearField(Field2d field) {
    FieldObject2d fieldObj = field.getObject("Auto Preview");
    fieldObj.setPoses(List.of());
  }
}
