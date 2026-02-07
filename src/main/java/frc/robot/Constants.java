package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // =========================================================================
    // STUDENT ADJUSTMENT AREA: CONTROLLER PORTS
    // =========================================================================
    public static final class OperatorConstants {
        // Port 0 is usually the "Driver" controller
        public static final int kDriverControllerPort = 0;

        // Port 1 is usually the "Operator" (Mechanisms) controller
        public static final int kOperatorControllerPort = 1;
    }

    // =========================================================================
    // STUDENT ADJUSTMENT AREA: MECHANISM IDS
    // =========================================================================
    public static final class MechanismIds {
        // TODO: STUDENTS - Update these IDs to match your REV Hardware Client / Phoenix
        // Tuner

        // Launcher (Shooter)
        public static final int kLauncherLeftID = 10;
        public static final int kLauncherRightID = 11;

        // Feeders
        public static final int kLaunchFeederID = 13;
        public static final int kFloorFeederID = 14;

        // Intake
        public static final int kIntakePivotID = 15;
        public static final int kIntakeFloorID = 16;

        // Climber
        public static final int kClimberID = 16;
    }
}
