package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.commands.IntakeFactory;
import frc.robot.commands.LauncherFactory;

// Subsystems Are Here again
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.LaunchFeeder;
import frc.robot.subsystems.FloorFeeder;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeFloor;
import frc.robot.subsystems.Climber;

public class RobotContainer {
        private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        /* Swerve requests */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1)
                        .withRotationalDeadband(MaxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        // =========================================================================
        // CONTROLLERS
        // =========================================================================
        private final CommandXboxController driverJoystick = new CommandXboxController(
                        Constants.OperatorConstants.kDriverControllerPort);

        private final CommandXboxController operatorJoystick = new CommandXboxController(
                        Constants.OperatorConstants.kOperatorControllerPort);

        // NOTE: You currently have testingJoystick on the SAME port as driverJoystick.
        // If you actually have a separate controller, give it its own port constant.
        private final CommandXboxController testingJoystick = new CommandXboxController(
                        Constants.OperatorConstants.kDriverControllerPort);

        // =========================================================================
        // TESTING CONSTANTS – EASY TO CHANGE (ALL AT 40%)
        // =========================================================================
        private static final double TEST_PERCENT = 0.40; // 40%

        // Choose safe caps (RPS). These are NOT "free speed"; they're safe test caps.
        private static final double INTAKE_FLOOR_MAX_RPS = 80.0; // Kraken X60-ish cap
        private static final double INTAKE_PIVOT_MAX_RPS = 15.0; // pivot should be slow
        private static final double FLOOR_FEEDER_MAX_RPS = 80.0; // feeder cap
        private static final double LAUNCH_FEEDER_MAX_RPS = 80.0; // feeder cap
        private static final double LAUNCHER_MAX_RPS = 80.0; // shooter cap

        private static final double INTAKE_FLOOR_TEST_RPS = TEST_PERCENT * INTAKE_FLOOR_MAX_RPS;
        private static final double INTAKE_PIVOT_TEST_RPS = TEST_PERCENT * INTAKE_PIVOT_MAX_RPS;
        private static final double FLOOR_FEEDER_TEST_RPS = TEST_PERCENT * FLOOR_FEEDER_MAX_RPS;
        private static final double LAUNCH_FEEDER_TEST_RPS = TEST_PERCENT * LAUNCH_FEEDER_MAX_RPS;
        private static final double LAUNCHER_TEST_RPS = TEST_PERCENT * LAUNCHER_MAX_RPS;

        // =========================================================================
        // SUBSYSTEMS
        // =========================================================================
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        public final Launcher launcher = new Launcher();
        public final LaunchFeeder launchFeeder = new LaunchFeeder();
        public final FloorFeeder floorFeeder = new FloorFeeder();
        public final IntakePivot intakePivot = new IntakePivot();
        public final IntakeFloor intakeFloor = new IntakeFloor();
        public final Climber climber = new Climber();

        public RobotContainer() {
                configureBindings();
        }

        private void configureBindings() {
                // =========================================================================
                // DRIVER CONTROLS (CHASSIS)
                // =========================================================================
                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(() -> drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed)
                                                .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed)
                                                .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate)));

                driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
                driverJoystick.leftBumper()
                                .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

                drivetrain.registerTelemetry(logger::telemeterize);

                // =========================================================================
                // OPERATOR CONTROLS (your existing mappings)
                // ========================================================================= //
                // Launcher & Feeder
                operatorJoystick.rightBumper().whileTrue(launcher.closeShotCommand());
                operatorJoystick.leftBumper().whileTrue(launchFeeder.feederInCommand());

                // Intake
                operatorJoystick.x().whileTrue(intakeFloor.intakeInCommand());
                operatorJoystick.b().whileTrue(intakeFloor.intakeOutCommand());

                operatorJoystick.y().onTrue(intakePivot.pivotToAmp());
                operatorJoystick.a().onTrue(intakePivot.pivotStow());

                // =========================================================================
                // TESTING JOYSTICK BINDINGS (all at 40%)
                // =========================================================================

                // IntakeFloor: POV left forward, POV right reverse (stop on release)
                testingJoystick.povLeft().whileTrue(intakeFloor.intakeCommand(INTAKE_FLOOR_TEST_RPS));
                testingJoystick.povRight().whileTrue(intakeFloor.intakeCommand(-INTAKE_FLOOR_TEST_RPS));
                testingJoystick.povLeft().onFalse(intakeFloor.stopCommand());
                testingJoystick.povRight().onFalse(intakeFloor.stopCommand());

                // IntakePivot: POV up forward, POV down reverse (stop on release)
                // Assumes IntakePivot has pivotRpsCommand(double) and stopCommand()
                testingJoystick.povUp().whileTrue(intakePivot.pivotRpsCommand(INTAKE_PIVOT_TEST_RPS));
                testingJoystick.povDown().whileTrue(intakePivot.pivotRpsCommand(-INTAKE_PIVOT_TEST_RPS));
                testingJoystick.povUp().onFalse(intakePivot.stopCommand());
                testingJoystick.povDown().onFalse(intakePivot.stopCommand());

                // FloorFeeder: left trigger forward, right trigger reverse (stop on release)
                testingJoystick.leftTrigger(0.1).whileTrue(floorFeeder.feederCommand(FLOOR_FEEDER_TEST_RPS));
                testingJoystick.rightTrigger(0.1).whileTrue(floorFeeder.feederCommand(-FLOOR_FEEDER_TEST_RPS));
                testingJoystick.leftTrigger(0.1).onFalse(floorFeeder.stopCommand());
                testingJoystick.rightTrigger(0.1).onFalse(floorFeeder.stopCommand());

                // LaunchFeeder: A forward, Y reverse (stop on release)
                testingJoystick.a().whileTrue(launchFeeder.feederCommand(LAUNCH_FEEDER_TEST_RPS));
                testingJoystick.y().whileTrue(launchFeeder.feederCommand(-LAUNCH_FEEDER_TEST_RPS));

                testingJoystick.a().onFalse(launchFeeder.stopCommand());
                testingJoystick.y().onFalse(launchFeeder.stopCommand());

                // Launcher flywheels: B at 40% (stop on release)
                testingJoystick.b().whileTrue(launcher.runShooterRpsCommand(LAUNCHER_TEST_RPS));
                testingJoystick.b().onFalse(launcher.runShooterRpsCommand(0.0));

                // =========================================================================
                // TESTING: LEFT BUMPER = Deploy intake + run intake chain @ 90%
                // - LaunchFeeder auto-stops when CANrange detects ball
                // - Other motors keep running until button released
                // =========================================================================
                testingJoystick.leftBumper().whileTrue(
                                IntakeFactory.deployAndIntakeChainPercent(
                                                intakePivot,
                                                intakeFloor,
                                                floorFeeder,
                                                launchFeeder,
                                                90.0, // deploy deg
                                                0.0, // stow deg on release
                                                0.90, // 90%
                                                80.0, // intakeFloor max RPS cap
                                                80.0, // floorFeeder max RPS cap
                                                80.0 // launchFeeder max RPS cap
                                ));

                // =========================================================================
                // TESTING: RIGHT BUMPER = Shooter + FloorFeeder + LaunchFeeder @ 90%
                // Stops all when released (handled inside LauncherFactory via finallyDo)
                // =========================================================================
                testingJoystick.rightBumper().whileTrue(
                                LauncherFactory.shootFeedPercent(
                                                launcher,
                                                floorFeeder,
                                                launchFeeder,
                                                0.90, // 90%
                                                80.0, // launcher max RPS cap
                                                80.0, // floor feeder max RPS cap
                                                80.0 // launch feeder max RPS cap
                                ));

                // Climber not mapped yet
                // operatorJoystick.start().whileTrue(climber.climbCommand());
        }

        public Command getAutonomousCommand() {
                final var idle = new SwerveRequest.Idle();
                return Commands.sequence(
                                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                                drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
                                                .withVelocityY(0)
                                                .withRotationalRate(0))
                                                .withTimeout(5.0),
                                drivetrain.applyRequest(() -> idle));
        }
}
