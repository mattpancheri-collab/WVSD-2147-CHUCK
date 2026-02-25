package frc.robot;

public final class Constants {
  private Constants() {
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
    public static final double kShooterVolts = 11.0;
    public static final double kFloorFeederVolts = 11.0;
    public static final double kLaunchFeederVolts = 11.0;

    // Delay before feeders start (seconds)
    public static final double kShooterSpinUpSeconds = 3.0;
  }

  // ===========================================================================
  // TESTING VOLTAGES
  // ===========================================================================
  public static final class TestingConstants {
    private TestingConstants() {
    }

    // 100% = 12V
    public static final double kTestVoltsShooter = 11.0;
    public static final double kTestVoltsFloorFeeder = 11.0;
    public static final double kTestVoltsLaunchFeeder = 11.0;
    public static final double kTestVoltsIntakeGround = 11.0;
    public static final double kTestVoltsIntakePivot = 11.0;
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

    public static final double kMinAngleDeg = 0.0;
    public static final double kMaxAngleDeg = 90.0;
    public static final double kDeployAngleDeg = 90.0;

    public static final double kP = 25.0;
    public static final double kI = 0.0;
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

    public static final double kP = 0.10;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kMaxRPS = 60.0;
    public static final double kIntakeInRPS = 25.0;
    public static final double kIntakeOutRPS = -25.0;

    public static final double kRampRPSPerSec = 200.0;

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

    public static final double kP = 0.10;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // Feedforward (optional) — REQUIRED because FloorFeeder.java references these
    public static final double kS = 0.0;
    public static final double kV = 0.12;
    public static final double kA = 0.0;

    public static final double kMaxRPS = 60.0;
    public static final double kFeedInRPS = 25.0;
    public static final double kFeedOutRPS = -25.0;

    public static final double kRampRPSPerSec = 200.0;

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

    public static final boolean kEnableCANrange = false;

    public static final double kBallDetectionDistanceMeters = 0.15;
    public static final double kBallDebounceSeconds = 0.08;

    public static final boolean kAutoStopOnBall = true;

    public static final double kP = 0.10;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kS = 0.0;
    public static final double kV = 0.12;
    public static final double kA = 0.0;

    public static final double kMaxRPS = 60.0;
    public static final double kFeedInRPS = 25.0;
    public static final double kFeedOutRPS = -25.0;

    public static final double kRampRPSPerSec = 200.0;

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

    public static final double kShooterP = 0.10;
    public static final double kShooterI = 0.0;
    public static final double kShooterD = 0.0;

    public static final double kShooterS = 0.0;
    public static final double kShooterV = 0.0;
    public static final double kShooterA = 0.0;

    public static final boolean kShooterEnableStatorLimit = true;
    public static final double kShooterStatorLimitAmps = 60.0;

    public static final double kShooterMaxRPS = 80.0;
    public static final double kShooterRampRPSPerSec = 200.0;

    public static final double kShooterIdleRPS = 10.0;
    public static final double kShooterCloseRPS = 55.0;
    public static final double kShooterFarRPS = 70.0;

    public static final double kHoodGearRatio = 100.0;

    public static final double kHoodP = 25.0;
    public static final double kHoodI = 0.0;
    public static final double kHoodD = 0.5;

    public static final double kHoodS = 0.0;
    public static final double kHoodG = 0.0;
    public static final double kHoodV = 0.0;
    public static final double kHoodA = 0.0;

    public static final boolean kHoodEnableStatorLimit = true;
    public static final double kHoodStatorLimitAmps = 40.0;

    public static final double kHoodMinDeg = 0.0;
    public static final double kHoodMaxDeg = 60.0;

    public static final double kHoodCloseDeg = 20.0;
    public static final double kHoodFarDeg = 45.0;
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
