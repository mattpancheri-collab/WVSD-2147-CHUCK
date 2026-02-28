package frc.robot;

public final class Constants {
  private Constants() {
  }

  // ===========================================================================
  // GLOBAL TUNING
  // ===========================================================================
  public static final class GlobalConstants {
    private GlobalConstants() {
    }

    public static final double kDefaultVoltage = 11.0;
    public static final double kDefaultRampRate = 200.0;
  }

  // ===========================================================================
  // LAUNCHER FACTORY
  // ===========================================================================
  public static final class ShootingConstants {
    private ShootingConstants() {
    }

    // Safety clamp
    public static final double kMaxVoltage = 11.0;

    // Polarity (+1 or -1)
    public static final double kShooterPolarity = -1.0;

    // Voltages (100% = 12V)
    public static final double kShooterVolts = GlobalConstants.kDefaultVoltage;
    public static final double kFloorFeederVolts = GlobalConstants.kDefaultVoltage;
    public static final double kLaunchFeederVolts = GlobalConstants.kDefaultVoltage;

    // Delay before feeders start (seconds)
    public static final double kShooterSpinUpSeconds = 2.0;
  }

  // ===========================================================================
  // TESTING VOLTAGES
  // ===========================================================================
  public static final class TestingConstants {
    private TestingConstants() {
    }

    // 100% = 12V
    public static final double kTestVoltsShooter = GlobalConstants.kDefaultVoltage;
    public static final double kTestVoltsFloorFeeder = GlobalConstants.kDefaultVoltage;
    public static final double kTestVoltsLaunchFeeder = GlobalConstants.kDefaultVoltage;
    public static final double kTestVoltsIntakeGround = GlobalConstants.kDefaultVoltage;
    public static final double kTestVoltsIntakePivot = GlobalConstants.kDefaultVoltage;
  }

  // ===========================================================================
  // CAN BUS
  // ===========================================================================
  public static final class BusConstants {
    public static final com.ctre.phoenix6.CANBus kDefaultBus = new com.ctre.phoenix6.CANBus("rio");
  }

  // ===========================================================================
  // CAN IDs (all devices in one place — prevents duplicate-ID bugs)
  // ===========================================================================
  public static final class CANConstants {
    private CANConstants() {
    }

    // Launcher / Shooter
    public static final int kShooterLeaderID = 9;
    public static final int kShooterFollower1ID = 10;
    public static final int kShooterFollower2ID = 11;
    public static final int kHoodID = 12;

    // Feeders
    public static final int kLaunchFeederID = 13;
    public static final int kFloorFeederID = 14;

    // Intake
    public static final int kPivotID = 15;
    public static final int kIntakeID = 16;

    // Climber
    public static final int kClimberID = 17;

    // Sensors
    public static final int kCANrangeID = 30;
  }

  // ===========================================================================
  // OPERATOR INTERFACE
  // ===========================================================================
  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kTestingControllerPort = 2;
  }

  // ===========================================================================
  // INTAKE PIVOT
  // ===========================================================================
  public static final class IntakePivotConstants {
    private IntakePivotConstants() {
    }

    // CAN ID is now in CANConstants.kPivotID

    public static final double kGearRatio = 100.0;

    public static final double kIntakeAngleDeg = 0.0;
    public static final double kIdleAngleDeg = 90.0;

    /* --- PID TUNING: Controls the Arm Movement --- */
    /**
     * Proportional Gain: The "Strength". Higher = faster response, but more
     * shaking.
     */
    public static final double kP = 25.0;
    /**
     * Integral Gain: The "History". Builds power over time to reach the target
     * exactly. Usually 0.
     */
    public static final double kI = 0.0;
    /**
     * Derivative Gain: The "Brake". Smooths out the stopping to prevent bouncing.
     */
    public static final double kD = 0.5;

    public static final double kMaxVelocityRPS = 5.0;
    public static final double kRampRPSPerSec = 20.0;

    public static final boolean kEnableStatorLimit = true;
    public static final double kStatorLimitAmps = 40.0;
  }

  // ===========================================================================
  // INTAKE GROUND (called IntakeFloorConstants in subsystem)
  // ===========================================================================
  public static final class IntakeFloorConstants {
    private IntakeFloorConstants() {
    }

    // CAN ID is now in CANConstants.kIntakeID
    public static final double kIntakeVolts = GlobalConstants.kDefaultVoltage;

    /* --- SPEED TUNING: Controls the Roller Speed --- */
    /**
     * Proportional Gain: High values help the motor stay at the set RPM when a ball
     * hits it.
     */
    public static final double kP = 0.10;
    /** Integral Gain: Usually 0. */
    public static final double kI = 0.0;
    /** Derivative Gain: Usually 0. */
    public static final double kD = 0.0;

    public static final double kMaxRPS = 60.0;
    public static final double kIntakeInRPS = 25.0;
    public static final double kIntakeOutRPS = -25.0;

    public static final double kRampRPSPerSec = GlobalConstants.kDefaultRampRate;

    public static final boolean kEnableStatorLimit = true;
    public static final double kStatorLimitAmps = 40.0;
  }

  // ===========================================================================
  // FLOOR FEEDER
  // ===========================================================================
  public static final class FloorFeederConstants {
    private FloorFeederConstants() {
    }

    // CAN ID is now in CANConstants.kFloorFeederID
    public static final double kIntakeVolts = GlobalConstants.kDefaultVoltage;

    /* --- PID & FEEDFORWARD TUNING: Controls Feed Speed --- */
    /** Proportional Gain: Main correction power to stay at target speed. */
    public static final double kP = 0.10;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    /**
     * Static Gain: Minimum voltage to overcome friction and get the motor spinning.
     */
    public static final double kS = 0.0;
    /**
     * Velocity Gain: Voltage required to maintain a specific speed (RPS). Main
     * speed driver.
     */
    public static final double kV = 0.12;
    /** Acceleration Gain: Extra "kick" of voltage when speeding up. */
    public static final double kA = 0.0;

    public static final double kMaxRPS = 60.0;
    public static final double kFeedInRPS = 25.0;
    public static final double kFeedOutRPS = -25.0;

    public static final double kRampRPSPerSec = GlobalConstants.kDefaultRampRate;

    public static final boolean kEnableStatorLimit = true;
    public static final double kStatorLimitAmps = 40.0;
  }

  // ===========================================================================
  // LAUNCH FEEDER
  // ===========================================================================
  public static final class LaunchFeederConstants {
    private LaunchFeederConstants() {
    }

    // CAN IDs are now in CANConstants.kLaunchFeederID / CANConstants.kCANrangeID
    public static final double kIntakeVolts = GlobalConstants.kDefaultVoltage;

    public static final boolean kEnableCANrange = false;

    public static final double kBallDetectionDistanceMeters = 0.15;
    public static final double kBallDebounceSeconds = 0.08;

    public static final boolean kAutoStopOnBall = true;

    /* --- PID & FEEDFORWARD TUNING: Controls Feed Speed --- */
    /**
     * Proportional Gain: Keeps the motor at the correct speed while feeding the
     * launcher.
     */
    public static final double kP = 0.10;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    /** Static Gain: Voltage to break friction. */
    public static final double kS = 0.0;
    /** Velocity Gain: Main driver for matching target RPS. */
    public static final double kV = 0.12;
    /** Acceleration Gain: Extra power during speed transitions. */
    public static final double kA = 0.0;

    public static final double kMaxRPS = 60.0;
    public static final double kFeedInRPS = 25.0;
    public static final double kFeedOutRPS = -25.0;

    public static final double kRampRPSPerSec = GlobalConstants.kDefaultRampRate;

    public static final boolean kEnableStatorLimit = true;
    public static final double kStatorLimitAmps = 40.0;
  }

  // ===========================================================================
  // LAUNCHER (SHOOTER + HOOD)
  // ===========================================================================
  public static final class LauncherConstants {
    private LauncherConstants() {
    }

    // CAN IDs are now in CANConstants (kShooterLeaderID, kShooterFollower1ID,
    // kShooterFollower2ID, kHoodID)

    /* --- SHOOTER SPEED TUNING --- */
    /**
     * Proportional Gain: High speed correction. If wheels slow down too much on
     * shot, increase this.
     */
    public static final double kShooterP = 0.10;
    public static final double kShooterI = 0.0;
    public static final double kShooterD = 0.0;

    /** Static Gain: Voltage to overcome motor friction. */
    public static final double kShooterS = 0.0;
    /**
     * Velocity Gain: Most important for shooter. Maps target RPS to base Voltage.
     */
    public static final double kShooterV = 0.0;
    /** Acceleration Gain: Helps the shooter spin up faster. */
    public static final double kShooterA = 0.0;

    public static final boolean kShooterEnableStatorLimit = true;
    public static final double kShooterStatorLimitAmps = 60.0;

    public static final double kShooterMaxRPS = 80.0;
    public static final double kShooterRampRPSPerSec = GlobalConstants.kDefaultRampRate;

    public static final double kShooterIdleRPS = 10.0;
    public static final double kShooterCloseRPS = 55.0;
    public static final double kShooterFarRPS = 70.0;

    public static final double kHoodGearRatio = 100.0;

    /* --- HOOD ANGLE TUNING --- */
    /** Proportional Gain: Main power to move the hood to a specific angle. */
    public static final double kHoodP = 25.0;
    public static final double kHoodI = 0.0;
    /** Derivative Gain: Dampens movement to prevent the hood from bouncing. */
    public static final double kHoodD = 0.5;

    /** Static Gain: Voltage to overcome gearbox friction. */
    public static final double kHoodS = 0.0;
    /**
     * Gravity Gain: Constant voltage to hold the hood up against its own weight.
     */
    public static final double kHoodG = 0.0;
    /** Velocity Gain: Voltage required for steady movement. */
    public static final double kHoodV = 0.0;
    /** Acceleration Gain: Extra power to start moving. */
    public static final double kHoodA = 0.0;

    public static final boolean kHoodEnableStatorLimit = true;
    public static final double kHoodStatorLimitAmps = 40.0;

    public static final double kHoodMinDeg = 0.0;
    public static final double kHoodMaxDeg = 60.0;

    public static final double kHoodCloseDeg = 20.0;
    public static final double kHoodFarDeg = 45.0;

    // Custom Button Mappings (B, X, Y)
    public static final double kHoodAngle1 = -10.0;
    public static final double kHoodAngle2 = -30.0;
    public static final double kHoodAngle3 = -45.0;
  }

  // ===========================================================================
  // CLIMBER
  // ===========================================================================
  public static final class ClimberConstants {
    private ClimberConstants() {
    }

    // CAN ID is now in CANConstants.kClimberID

    // 100% output
    public static final double kClimbPower = 1.0;
    public static final double kClimbReversePower = -1.0;

    public static final boolean kEnableStatorLimit = true;
    public static final double kStatorLimitAmps = 60.0;
  }
}
