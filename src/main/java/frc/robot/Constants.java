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

    public static final double kDefaultVoltage = 12.0;
    public static final double kDefaultRampRate = 200.0;
  }

  // ===========================================================================
  // LAUNCHER FACTORY
  // ===========================================================================
  public static final class ShootingConstants {
    private ShootingConstants() {
    }

    // Safety clamp
    public static final double kMaxVoltage = 12.0;

    // Polarity (+1 or -1)
    public static final double kShooterPolarity = -1.0;

    // Voltages (100% = 12V)
    public static final double kShooterVolts = 12.0;
    public static final double kFloorFeederVolts = 12.0;
    public static final double kLaunchFeederVolts = 12.0;

    // Delay before feeders start (seconds)
    public static final double kShooterSpinUpSeconds = 0.5;
  }

  // ===========================================================================
  // TESTING VOLTAGES
  // ===========================================================================
  public static final class TestingConstants {
    private TestingConstants() {
    }

    // 100% = 12V
    public static final double kTestVoltsShooter = 12.0;
    public static final double kTestVoltsFloorFeeder = 12.0;
    public static final double kTestVoltsLaunchFeeder = 12.0;
    public static final double kTestVoltsIntakeGround = 12.0;
    public static final double kTestVoltsIntakePivot = 12.0;
  }

  // ===========================================================================
  // CAN BUS
  // ===========================================================================
  public static final class BusConstants {
    public static final com.ctre.phoenix6.CANBus kDefaultBus =
        new com.ctre.phoenix6.CANBus("rio");
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
    public static final int kPivotFollowerID = 19;

    // Climber
    public static final int kClimberID = 17;

    // Sensors
    public static final int kCANrangeID = 31;
    public static final int kHoodRangeID = 30;
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

    public static final double kIntakeAngleDeg = 45.0;
    public static final double kIdleAngleDeg = 0.0;

    /* --- PID TUNING: Controls the Arm Movement --- */
    public static final double kP = 120.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kG = 0.75;

    public static final double kMaxVelocityRPS = 10.0;
    public static final double kRampRPSPerSec = 42.0;

    public static final boolean kEnableStatorLimit = true;
    public static final double kStatorLimitAmpsUp = 80.0;
    public static final double kStatorLimitAmpsDown = 30.0;
  }

  // ===========================================================================
  // INTAKE GROUND (called IntakeFloorConstants in subsystem)
  // ===========================================================================
  public static final class IntakeFloorConstants {
    private IntakeFloorConstants() {
    }

    // CAN ID is now in CANConstants.kIntakeID
    public static final double kIntakeVolts = 12.0;

    /* --- SPEED TUNING: Controls the Roller Speed --- */
    public static final double kP = 0.10;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kMaxRPS = 60.0;
    public static final double kIntakeInRPS = 25.0;
    public static final double kIntakeOutRPS = -25.0;

    public static final double kRampRPSPerSec = GlobalConstants.kDefaultRampRate;

    public static final boolean kEnableStatorLimit = true;
    public static final double kStatorLimitAmps = 80.0;
  }

  // ===========================================================================
  // FLOOR FEEDER
  // ===========================================================================
  public static final class FloorFeederConstants {
    private FloorFeederConstants() {
    }

    // CAN ID is now in CANConstants.kFloorFeederID
    public static final double kIntakeVolts = 12.0;

    /* --- PID & FEEDFORWARD TUNING: Controls Feed Speed --- */
    public static final double kP = 0.10;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kS = 0.0;
    public static final double kV = 0.12;
    public static final double kA = 0.0;

    public static final double kMaxRPS = 80.0;
    public static final double kFeedInRPS = 100.0;//was 80
    public static final double kFeedOutRPS = -25.0;

    public static final double kRampRPSPerSec = GlobalConstants.kDefaultRampRate;

    public static final boolean kEnableStatorLimit = true;
    public static final double kStatorLimitAmps = 80.0;
  }

  // ===========================================================================
  // LAUNCH FEEDER
  // ===========================================================================
  public static final class LaunchFeederConstants {
    private LaunchFeederConstants() {
    }

    // CAN IDs are now in CANConstants.kLaunchFeederID / CANConstants.kCANrangeID
    public static final double kIntakeVolts = 12.0;

    public static final boolean kEnableCANrange = false;

    public static final double kBallDetectionDistanceMeters = 0.15;
    public static final double kBallDebounceSeconds = 0.08;

    public static final boolean kAutoStopOnBall = true;

    /* --- PID & FEEDFORWARD TUNING: Controls Feed Speed --- */
    public static final double kP = 0.20;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kS = 0.0;
    public static final double kV = 0.12;
    public static final double kA = 0.0;

    public static final double kMaxRPS = 90.0;
    public static final double kFeedInRPS = 80.0; //was 68
    public static final double kFeedOutRPS = -80.0;

    public static final double kRampRPSPerSec = GlobalConstants.kDefaultRampRate;

    public static final boolean kEnableStatorLimit = true;
    public static final double kStatorLimitAmps = 80.0;
  }

  // ===========================================================================
  // LAUNCHER (SHOOTER + HOOD)
  // ===========================================================================
  public static final class LauncherConstants {
    private LauncherConstants() {
    }
  
    public static final double kShooterMaxRPS = 76.0;
    public static final double kShooterIdleRPS = 10.0;
    public static final double kShooterCloseRPS = 72; //close is 72
    public static final double kShooterFarRPS = 75.0;
    public static final double kShooterRampRPSPerSec = 300.0;
  
    public static final double kShooterP = 0.40;
    public static final double kShooterI = 0.0;
    public static final double kShooterD = 0.0;
  
    public static final double kShooterS = 0.20;
    public static final double kShooterV = 0.130;
    public static final double kShooterA = 0.0;
  
    public static final boolean kShooterEnableStatorLimit = true;
    public static final double kShooterStatorLimitAmps = 160.0;
  
    public static final double kShooterShotBoostVolts = 1.15;
    public static final double kShooterReadyToleranceRps = 2.0;
  
    public static final double kShooterFeedEnableErrorRps = 4.0;
    public static final double kShooterFeedDisableErrorRps = 6.0;
  
    public static final double kShooterPreHitBoostVolts = 1.35;
    public static final double kShooterPreHitBoostTimeSec = 0.10;
  
    public static final double kHoodGearRatio = 100.0;
  
    public static final double kHoodP = 2.0;
    public static final double kHoodI = 0.0;
    public static final double kHoodD = 0.5;
  
    public static final double kHoodG = 0.1;
  
    public static final double kHoodS = 0.0;
    public static final double kHoodV = 0.0;
    public static final double kHoodA = 0.0;
  
    public static final boolean kHoodEnableStatorLimit = true;
    public static final double kHoodStatorLimitAmps = 15.0;
    public static final double kHoodMinDeg = 0.0;
    public static final double kHoodMaxDeg = 250.0;
  
    public static final double kHoodAngle1 = 221.1;
    public static final double kHoodAngle2 = 0.0;
    public static final double kHoodAngle3 = 72.4;
  }

  // ===========================================================================
  // CLIMBER
  // ===========================================================================
  public static final class ClimberConstants {
    private ClimberConstants() {
    }

    // CAN ID is now in CANConstants.kClimberID
    public static final double kClimbPower = 1.0;
    public static final double kClimbReversePower = -1.0;

    public static final boolean kEnableStatorLimit = true;
    public static final double kStatorLimitAmps = 80.0;
  }
}
