package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// Subsystems
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.LaunchFeeder;
import frc.robot.subsystems.FloorFeeder;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeGround;
import frc.robot.subsystems.Climber;

// Factories
import frc.robot.commands.IntakeFactory;
import frc.robot.commands.LauncherFactory;

public class RobotContainer {

        private final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        // Slow mode parameters for Right Bumper
        private final double SlowMaxSpeed = 0.3 * MaxSpeed;
        private final double SlowMaxAngularRate = 0.3 * MaxAngularRate;

        // Swerve requests
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1)
                        .withRotationalDeadband(MaxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final SwerveRequest.FieldCentric slowDrive = new SwerveRequest.FieldCentric()
                        .withDeadband(SlowMaxSpeed * 0.1)
                        .withRotationalDeadband(SlowMaxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final Telemetry logger = new Telemetry(MaxSpeed);

        // Controllers
        private final CommandXboxController driverJoystick = new CommandXboxController(
                        Constants.OperatorConstants.kDriverControllerPort);

        private final CommandXboxController operatorJoystick = new CommandXboxController(
                        Constants.OperatorConstants.kOperatorControllerPort);

        private final CommandXboxController testingJoystick = new CommandXboxController(
                        Constants.OperatorConstants.kTestingControllerPort);

        // Subsystems
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        public final Launcher launcher = new Launcher();
        public final LaunchFeeder launchFeeder = new LaunchFeeder();
        public final FloorFeeder floorFeeder = new FloorFeeder();
        public final IntakePivot intakePivot = new IntakePivot();
        public final IntakeGround intakeGround = new IntakeGround();
        public final Climber climber = new Climber();

        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
                registerNamedCommands();
                configureBindings();

                // ---------------------------------------------------------------------------
                // PATHPLANNER AUTO CHOOSER
                // ---------------------------------------------------------------------------
                // AutoBuilder automatically finds all autonomous routines created in the
                // PathPlanner GUI and builds a SendableChooser. By putting it on the
                // SmartDashboard, drivers can select the desired auto on the driver station.
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        private void registerNamedCommands() {
                // ---------------------------------------------------------------------------
                // PATHPLANNER NAMED COMMANDS
                // ---------------------------------------------------------------------------
                // Register Named Commands here so PathPlanner can trigger them during auto.
                // The first argument (e.g. "Intake") MUST perfectly match the string used in
                // the PathPlanner GUI. The second argument is the WPILib Command to run.

                NamedCommands.registerCommand("Intake",
                                IntakeFactory.deployAndIntakeChainVoltage(intakePivot, intakeGround, floorFeeder,
                                                launchFeeder));

                NamedCommands.registerCommand("Shoot",
                                LauncherFactory.shootFeedVoltage(launcher, floorFeeder, launchFeeder));

                NamedCommands.registerCommand("Stow",
                                intakePivot.runOnce(() -> intakePivot
                                                .setAngleDegrees(Constants.IntakePivotConstants.kIdleAngleDeg)));
        }

        // ---------------------------------------------------------------------------
        // HELPERS
        // ---------------------------------------------------------------------------

        /** Simple helper for voltage-hold buttons */
        private static Command holdVolts(
                        edu.wpi.first.wpilibj2.command.Subsystem req,
                        Runnable start,
                        Runnable end) {
                return Commands.startEnd(start, end, req);
        }

        private void configureBindings() {

                // =========================================================================
                // DRIVER (SWERVE)
                // =========================================================================
                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(-driverJoystick.getLeftY() * MaxSpeed)
                                                .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed)
                                                .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate)));

                // Right Bumper: Slow Driving Mode
                driverJoystick.rightBumper().whileTrue(
                                drivetrain.applyRequest(() -> slowDrive
                                                .withVelocityX(-driverJoystick.getLeftY() * SlowMaxSpeed)
                                                .withVelocityY(-driverJoystick.getLeftX() * SlowMaxSpeed)
                                                .withRotationalRate(-driverJoystick.getRightX() * SlowMaxAngularRate)));

                // Triggers and Bumpers: Factory Commands
                driverJoystick.leftBumper().whileTrue(
                                IntakeFactory.deployAndIntakeChainVoltage(intakePivot, intakeGround, floorFeeder,
                                                launchFeeder));
                driverJoystick.rightTrigger().whileTrue(
                                LauncherFactory.shootFeedVoltage(launcher, floorFeeder, launchFeeder));

                // A Button: Gyro Reset
                driverJoystick.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

                // Placeholders (Mapped to Print commands so they don't do nothing and can be
                // identified easily)
                driverJoystick.leftTrigger().onTrue(Commands.print("Placeholder: Left Trigger"));
                driverJoystick.b().onTrue(Commands.print("Placeholder: B Button"));
                driverJoystick.x().onTrue(Commands.print("Placeholder: X Button"));
                driverJoystick.y().onTrue(Commands.print("Placeholder: Y Button"));
                driverJoystick.povUp().onTrue(Commands.print("Placeholder: Servo Loose"));
                driverJoystick.povDown().onTrue(Commands.print("Placeholder: Servo Break"));
                driverJoystick.povLeft().onTrue(Commands.print("Placeholder: POV Left"));
                driverJoystick.povRight().onTrue(Commands.print("Placeholder: POV Right"));

                drivetrain.registerTelemetry(logger::telemeterize);

                // =========================================================================
                // OPERATOR
                // =========================================================================
                operatorJoystick.rightBumper().whileTrue(launcher.closeShotCommand());
                operatorJoystick.leftBumper().whileTrue(launchFeeder.feederInCommand());

                operatorJoystick.x().whileTrue(intakeGround.intakeInCommand());
                operatorJoystick.b().whileTrue(intakeGround.intakeOutCommand());

                operatorJoystick.y().onTrue(intakePivot.runOnce(
                                () -> intakePivot.setAngleDegrees(Constants.IntakePivotConstants.kIdleAngleDeg)));
                operatorJoystick.a().onTrue(intakePivot.runOnce(
                                () -> intakePivot.setAngleDegrees(Constants.IntakePivotConstants.kIntakeAngleDeg)));

                // IntakeGround
                testingJoystick.povLeft().whileTrue(
                                holdVolts(intakeGround,
                                                () -> intakeGround.setVoltage(
                                                                +Constants.IntakeFloorConstants.kIntakeVolts),
                                                () -> intakeGround.setVoltage(0.0)));

                testingJoystick.povRight().whileTrue(
                                holdVolts(intakeGround,
                                                () -> intakeGround.setVoltage(
                                                                -Constants.IntakeFloorConstants.kIntakeVolts),
                                                () -> intakeGround.setVoltage(0.0)));

                // IntakePivot
                testingJoystick.povUp().whileTrue(
                                holdVolts(intakePivot,
                                                () -> intakePivot.setVoltage(
                                                                +Constants.TestingConstants.kTestVoltsIntakePivot),
                                                () -> intakePivot.setVoltage(0.0)));

                testingJoystick.povDown().whileTrue(
                                holdVolts(intakePivot,
                                                () -> intakePivot.setVoltage(
                                                                -Constants.TestingConstants.kTestVoltsIntakePivot),
                                                () -> intakePivot.setVoltage(0.0)));

                // FloorFeeder
                testingJoystick.leftTrigger(0.1).whileTrue(
                                holdVolts(floorFeeder,
                                                () -> floorFeeder.setVoltage(
                                                                +Constants.FloorFeederConstants.kIntakeVolts),
                                                () -> floorFeeder.setVoltage(0.0)));

                testingJoystick.rightTrigger(0.1).whileTrue(
                                holdVolts(floorFeeder,
                                                () -> floorFeeder.setVoltage(
                                                                -Constants.FloorFeederConstants.kIntakeVolts),
                                                () -> floorFeeder.setVoltage(0.0)));

                // LaunchFeeder
                testingJoystick.a().whileTrue(
                                holdVolts(launchFeeder,
                                                () -> launchFeeder.setVoltage(
                                                                Constants.LaunchFeederConstants.kIntakeVolts),
                                                () -> launchFeeder.setVoltage(0.0)));

                testingJoystick.y().whileTrue(
                                holdVolts(launchFeeder,
                                                () -> launchFeeder.setVoltage(
                                                                -Constants.LaunchFeederConstants.kIntakeVolts),
                                                () -> launchFeeder.setVoltage(0.0)));

                // Launcher
                testingJoystick.b().whileTrue(
                                holdVolts(launcher,
                                                () -> launcher.setVoltage(MathUtil.clamp(
                                                                Constants.TestingConstants.kTestVoltsShooter, -12, 12)),
                                                () -> launcher.setVoltage(0.0)));

                // =========================================================================
                // FACTORY COMMANDS
                // =========================================================================
                testingJoystick.leftBumper().whileTrue(
                                IntakeFactory.deployAndIntakeChainVoltage(
                                                intakePivot, intakeGround, floorFeeder, launchFeeder));

                testingJoystick.rightBumper().whileTrue(
                                LauncherFactory.shootFeedVoltage(
                                                launcher, floorFeeder, launchFeeder));
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
