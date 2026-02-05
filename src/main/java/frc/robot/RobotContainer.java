package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
// Import our new subsystems
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.LaunchFeeder;
import frc.robot.subsystems.FloorFeeder;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeFloor;
import frc.robot.subsystems.Climber;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // =========================================================================
    // CONTROLLERS
    // =========================================================================
    private final CommandXboxController driverJoystick = new CommandXboxController(
            Constants.OperatorConstants.kDriverControllerPort);
    private final CommandXboxController operatorJoystick = new CommandXboxController(
            Constants.OperatorConstants.kOperatorControllerPort);

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
        // DRIVER CONTROLS (Chassis)
        // =========================================================================
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed)
                        .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate)));

        driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverJoystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // =========================================================================
        // STUDENT ADJUSTMENT AREA: OPERATOR BINDINGS
        // =========================================================================
        // TODO: STUDENTS - Map your Xbox buttons to Subsystem commands here!
        // Example: operatorJoystick.a().whileTrue(launcher.runLauncherCommand());

        // Launcher & Feeder
        operatorJoystick.rightBumper().whileTrue(launcher.runLauncherCommand());
        operatorJoystick.leftBumper().whileTrue(launchFeeder.feedCommand());

        // Intake
        operatorJoystick.x().whileTrue(intakeFloor.runIntakeCommand());
        operatorJoystick.b().whileTrue(intakeFloor.outtakeCommand());

        // Intake Pivot (Manual example)
        operatorJoystick.y().onTrue(intakePivot.pivotToAmp());
        operatorJoystick.a().onTrue(intakePivot.pivotStow());

        // Climber
        // WAITING FOR STUDENTS TO DECIDE BUTTONS
        // operatorJoystick.start().whileTrue(climber.climbCommand());
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
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
