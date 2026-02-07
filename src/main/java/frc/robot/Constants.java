package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose.
 * All constants should be declared globally (i.e. public static).
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 *
 * <p>
 * STUDENTS: This is your central location for tuning the robot!
 * Change CAN IDs, speeds, and PID values here - they will automatically
 * apply to all subsystems.
 */
public final class Constants {

    // =========================================================================
    // CONTROLLER PORTS
    // =========================================================================
    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    // =========================================================================
    // CAN BUS CONFIGURATION
    // =========================================================================
    public static final class CANBus {
        /** ✅ Default CAN bus object (RoboRIO CAN) */
        public static final com.ctre.phoenix6.CANBus kDefaultBus = new com.ctre.phoenix6.CANBus("rio");
    }

    // =========================================================================
    // LAUNCHER (SHOOTER) - 4 MOTORS TOTAL
    // =========================================================================
    public static final class LauncherConstants {
        // CAN IDs
        /** ✅ CAN ID for the shooter leader (Kraken X60) */
        public static final int kShooterLeaderID = 9;
        /** ✅ CAN ID for shooter follower 1 (Kraken X60) */
        public static final int kShooterFollower1ID = 10;
        /** ✅ CAN ID for shooter follower 2 (Kraken X60) */
        public static final int kShooterFollower2ID = 11;
        /** ✅ CAN ID for the hood position motor (Kraken X44) */
        public static final int kHoodID = 12;

        // Shooter Velocity PID (tune these!)
        /** 📈 Proportional gain for shooter velocity PID */
        public static final double kShooterP = 0.10;
        /** 📈 Integral gain for shooter velocity PID */
        public static final double kShooterI = 0.0;
        /** 📈 Derivative gain for shooter velocity PID */
        public static final double kShooterD = 0.0;
        /** ⚡ Static friction feedforward (volts) */
        public static final double kShooterS = 0.0;
        /** ⚡ Velocity feedforward (volts / (rotations/sec)) */
        public static final double kShooterV = 0.0;
        /** ⚡ Acceleration feedforward (volts / (rotations/sec^2)) */
        public static final double kShooterA = 0.0;

        // Shooter Speed Presets (RPS)
        /** 🏎️ Maximum safe speed for flywheels (Rotations Per Second). Range: 0-90. */
        public static final double kShooterMaxRPS = 85.0;
        /** 🏎️ Speed used when "idling" the launcher (Rotations Per Second) */
        public static final double kShooterIdleRPS = 10.0;
        /** 🏎️ Speed for a close-range shot (Rotations Per Second) */
        public static final double kShooterCloseRPS = 45.0;
        /** 🏎️ Speed for a far-range shot (Rotations Per Second) */
        public static final double kShooterFarRPS = 65.0;
        /**
         * 📉 How fast the motors ramp up/down (RPS per second). Higher = more
         * aggressive.
         */
        public static final double kShooterRampRPSPerSec = 250.0;

        // Shooter Current Limiting
        /** 🛡️ Whether to enable stator current limiting for the shooter leader */
        public static final boolean kShooterEnableStatorLimit = true;
        /** 🛡️ Stator current limit for flywheels (Amps). Range: 40-100. */
        public static final double kShooterStatorLimitAmps = 80.0;

        // Hood Position PID (tune these!)
        /** 📈 Proportional gain for hood position PID. Range: 10-100. */
        public static final double kHoodP = 60.0;
        /** 📈 Integral gain for hood position PID */
        public static final double kHoodI = 0.0;
        /** 📈 Derivative gain for hood position PID */
        public static final double kHoodD = 0.0;
        /** ⚡ Static friction feedforward for hood (volts) */
        public static final double kHoodS = 0.0;
        /**
         * 🌍 Gravity compensation (volts). Increase if the hood droops at low angles.
         */
        public static final double kHoodG = 0.0;
        /** ⚡ Velocity feedforward for hood position (volts / (rotations/sec)) */
        public static final double kHoodV = 0.0;
        /** ⚡ Acceleration feedforward for hood position (volts / (rotations/sec^2)) */
        public static final double kHoodA = 0.0;

        // Hood Angle Limits (degrees)
        /** 📏 Minimum physical hood angle (degrees). Usually 0 (stowed). */
        public static final double kHoodMinDeg = 0.0;
        /** 📏 Maximum physical hood angle (degrees). Usually 60. */
        public static final double kHoodMaxDeg = 60.0;
        /** 📏 Hood angle for close-range shot (degrees) */
        public static final double kHoodCloseDeg = 20.0;
        /** 📏 Hood angle for far-range shot (degrees) */
        public static final double kHoodFarDeg = 40.0;

        // Hood Gear Ratio (motor rotations / hood rotations)
        /** ⚙️ Gear ratio of the hood mechanism (motor:mechanism). */
        public static final double kHoodGearRatio = 36.57;

        // Hood Current Limiting
        /** 🛡️ Whether to enable stator current limiting for the hood Kraken X44 */
        public static final boolean kHoodEnableStatorLimit = true;
        /** 🛡️ Stator current limit for hood (Amps). Range: 20-60. */
        public static final double kHoodStatorLimitAmps = 40.0;
    }

    // =========================================================================
    // LAUNCH FEEDER (feeds balls into shooter)
    // =========================================================================
    public static final class LaunchFeederConstants {
        // CAN IDs
        /** ✅ CAN ID for the launch feeder motor (TalonFX) */
        public static final int kFeederID = 13;
        /** ✅ CAN ID for the CANrange distance sensor (beam-break replacement) */
        public static final int kCANrangeID = 30;

        // Velocity PID (tune these!)
        /** 📈 Proportional gain for feeder velocity PID */
        public static final double kP = 0.10;
        /** 📈 Integral gain for feeder velocity PID */
        public static final double kI = 0.0;
        /** 📈 Derivative gain for feeder velocity PID */
        public static final double kD = 0.0;

        // Speed Presets (RPS)
        /** 🏎️ Maximum safe speed for feeder (Rotations Per Second) */
        public static final double kMaxRPS = 80.0;
        /** 🏎️ Speed for feeding balls "in" to the launcher (Rotations Per Second) */
        public static final double kFeedInRPS = 40.0;
        /** 🏎️ Speed for spitting balls "out" (Rotations Per Second) */
        public static final double kFeedOutRPS = -20.0;
        /** 📉 How fast the feeder motor ramps (RPS per second) */
        public static final double kRampRPSPerSec = 200.0;

        // Ball Detection
        /**
         * 📏 Distance (meters) to trigger ball detection in the feeder. Range:
         * 0.05-0.20.
         */
        public static final double kBallDetectionDistanceMeters = 0.10;
        /**
         * ⏱️ Time (seconds) the ball must be detected before acting. Prevents false
         * triggers.
         */
        public static final double kBallDebounceSeconds = 0.05;

        /** 🛡️ Whether the feeder should automatically stop when a ball is detected */
        public static final boolean kAutoStopOnBall = true;

        // Current Limiting
        /** 🛡️ Whether to enable stator current limiting for the feeder TalonFX */
        public static final boolean kEnableStatorLimit = true;
        /** 🛡️ Stator current limit for feeder (Amps). Range: 40-70. */
        public static final double kStatorLimitAmps = 60.0;
    }

    // =========================================================================
    // FLOOR FEEDER (moves balls from intake to launcher feeder)
    // =========================================================================
    public static final class FloorFeederConstants {
        // CAN ID
        /** ✅ CAN ID for the floor feeder motor (TalonFX) */
        public static final int kFeederID = 14;

        // Velocity PID (tune these!)
        /** 📈 Proportional gain for floor feeder velocity PID */
        public static final double kP = 0.10;
        /** 📈 Integral gain for floor feeder velocity PID */
        public static final double kI = 0.0;
        /** 📈 Derivative gain for floor feeder velocity PID */
        public static final double kD = 0.0;

        // Speed Presets (RPS)
        /** 🏎️ Maximum safe speed for floor feeder (Rotations Per Second) */
        public static final double kMaxRPS = 80.0;
        /** 🏎️ Speed for moving balls forward (Rotations Per Second) */
        public static final double kFeedInRPS = 40.0;
        /** 🏎️ Speed for clearing jams (Rotations Per Second) */
        public static final double kFeedOutRPS = -20.0;
        /** 📉 How fast the floor feeder motor ramps (RPS per second) */
        public static final double kRampRPSPerSec = 200.0;

        // Current Limiting
        /** 🛡️ Whether to enable stator current limiting for the TalonFX */
        public static final boolean kEnableStatorLimit = true;
        /** 🛡️ Stator current limit for floor feeder (Amps). Range: 40-70. */
        public static final double kStatorLimitAmps = 60.0;
    }

    // =========================================================================
    // INTAKE PIVOT (rotates intake in/out)
    // =========================================================================
    public static final class IntakePivotConstants {
        // CAN ID
        /** ✅ CAN ID for the intake pivot motor (Kraken) */
        public static final int kPivotID = 15;

        // Position PID (tune these!)
        /** 📈 Proportional gain for pivot position PID. Range: 20-80. */
        public static final double kP = 40.0;
        /** 📈 Integral gain for pivot position PID */
        public static final double kI = 0.0;
        /** 📈 Derivative gain for pivot position PID */
        public static final double kD = 0.5;
        /** 🌍 Gravity compensation (volts). Crucial for holding the intake up. */
        public static final double kG = 0.35;
        /** ⚡ Velocity feedforward (volts / (rotations/sec)) */
        public static final double kV = 0.0;
        /** ⚡ Acceleration feedforward (volts / (rotations/sec^2)) */
        public static final double kA = 0.0;

        // Angle Limits (degrees)
        /** 📏 Minimum physical pivot angle (degrees). 0 = Stowed. */
        public static final double kMinAngleDeg = 0.0;
        /** 📏 Maximum physical pivot angle (degrees). 120 = Over-extended. */
        public static final double kMaxAngleDeg = 120.0;
        /** 📏 Angle for the stowed position (degrees) */
        public static final double kStowAngleDeg = 0.0;
        /** 📏 Angle for the standard floor intake position (degrees) */
        public static final double kDeployAngleDeg = 90.0;
        /** 📏 Angle for scoring in the AMP (degrees) */
        public static final double kAmpAngleDeg = 45.0;

        // Gear Ratio (motor rotations / pivot rotations)
        /**
         * ⚙️ Gear ratio of the pivot (Intake rotates once for every 50 motor rotations)
         */
        public static final double kGearRatio = 50.0;

        // Current Limiting
        /** 🛡️ Whether to enable stator current limiting for the Kraken */
        public static final boolean kEnableStatorLimit = true;
        /** 🛡️ Stator current limit for the pivot (Amps). Range: 20-50. */
        public static final double kStatorLimitAmps = 40.0;

        // Velocity Limit for manual control (RPS)
        /**
         * 🏎️ Maximum rotation speed for the pivot mechanism (RPS). Keep low for
         * safety.
         */
        public static final double kMaxVelocityRPS = 15.0;
    }

    // =========================================================================
    // INTAKE FLOOR (intake rollers)
    // =========================================================================
    public static final class IntakeFloorConstants {
        // CAN ID
        /** ✅ CAN ID for the floor intake roller motor (TalonFX) */
        public static final int kIntakeID = 16;

        // Velocity PID (tune these!)
        /** 📈 Proportional gain for intake roller velocity PID */
        public static final double kP = 0.10;
        /** 📈 Integral gain for intake roller velocity PID */
        public static final double kI = 0.0;
        /** 📈 Derivative gain for intake roller velocity PID */
        public static final double kD = 0.0;

        // Speed Presets (RPS)
        /** 🏎️ Maximum safe speed for intake rollers (Rotations Per Second) */
        public static final double kMaxRPS = 80.0;
        /** 🏎️ Speed for active intaking (Rotations Per Second) */
        public static final double kIntakeInRPS = 60.0;
        /** 🏎️ Speed for ejecting game pieces (Rotations Per Second) */
        public static final double kIntakeOutRPS = -30.0;
        /** 📉 How fast the intake rollers ramp (RPS per second) */
        public static final double kRampRPSPerSec = 200.0;

        // Current Limiting
        /** 🛡️ Whether to enable stator current limiting for the TalonFX */
        public static final boolean kEnableStatorLimit = true;
        /** 🛡️ Stator current limit for intake (Amps). Range: 40-70. */
        public static final double kStatorLimitAmps = 60.0;
    }

    // =========================================================================
    // CLIMBER
    // =========================================================================
    public static final class ClimberConstants {
        // CAN ID (changed from 16 to 17 to avoid conflict with IntakeFloor)
        /** ✅ CAN ID for the climber motor (TalonFX) */
        public static final int kClimberID = 17;

        // Climb Power (percent output)
        /** 🔋 Percent output for climbing (-1.0 to 1.0). */
        public static final double kClimbPower = 0.8;
        /** 🔋 Percent output for reversing the climber (-1.0 to 1.0). */
        public static final double kClimbReversePower = -0.8;

        // Current Limiting
        /** 🛡️ Whether to enable stator current limiting for the TalonFX */
        public static final boolean kEnableStatorLimit = true;
        /** 🛡️ Stator current limit for climber (Amps). Range: 40-100. */
        public static final double kStatorLimitAmps = 80.0;
    }
}
