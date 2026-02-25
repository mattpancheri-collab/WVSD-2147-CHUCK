package frc.robot.commands;

/**
 * Factory rules:
 * - static methods only
 * - DO NOT store subsystems as fields
 * - DO NOT read joysticks here
 * - ONLY build and return Commands
 */
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;

public final class AutoFactory {
  private AutoFactory() {
  }

  /**
   * Example: A named auto created in the PathPlanner GUI.
   * Make sure the string matches the name of the file in
   * src/main/deploy/pathplanner/autos/
   */
  public static Command exampleAuto() {
    return new PathPlannerAuto("Example Auto");
  }
}