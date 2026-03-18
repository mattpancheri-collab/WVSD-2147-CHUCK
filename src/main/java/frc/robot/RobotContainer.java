package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Generated.TunerConstants;
import frc.robot.commands.IntakeFactory;
import frc.robot.commands.LauncherFactory;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FloorFeeder;
import frc.robot.subsystems.IntakeGround;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.LaunchFeeder;
import frc.robot.subsystems.Launcher;

public class RobotContainer {
  private final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  // Slow mode parameters for Right Bumper
  private final double SlowMaxSpeed = 0.3 * MaxSpeed;
  private final double SlowMaxAngularRate = 0.3 * MaxAngularRate;

  // Swerve requests
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.FieldCentric slowDrive =
      new SwerveRequest.FieldCentric()
          .withDeadband(SlowMaxSpeed * 0.1)
          .withRotationalDeadband(SlowMaxAngularRate * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  // Controllers
  private final CommandXboxController driverJoystick =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
  private final CommandXboxController operatorJoystick =
      new CommandXboxController(Constants.OperatorConstants.kOperatorControllerPort);
  private final CommandXboxController testingJoystick =
      new CommandXboxController(Constants.OperatorConstants.kTestingControllerPort);

  // Subsystems
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final Launcher launcher = new Launcher();
  public final LaunchFeeder launchFeeder = new LaunchFeeder();
  public final FloorFeeder floorFeeder = new FloorFeeder();
  public final IntakePivot intakePivot = new IntakePivot();
  public final IntakeGround intakeGround = new IntakeGround();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    registerNamedCommands();
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand(
        "IntakeFactory",
        IntakeFactory.intakeOnlyCommand(intakeGround, intakePivot));

    NamedCommands.registerCommand(
        "LauncherFactory",
        LauncherFactory.shootFeedVelocity(
            launcher, floorFeeder, launchFeeder, intakeGround, intakePivot));

    NamedCommands.registerCommand(
        "Stow",
        intakePivot.runOnce(
            () -> intakePivot.setAngleDegrees(Constants.IntakePivotConstants.kIdleAngleDeg)));
  }

  private static Command holdVolts(
      edu.wpi.first.wpilibj2.command.Subsystem req, Runnable start, Runnable end) {
    return Commands.startEnd(start, end, req);
  }

  private void configureBindings() {
    // DRIVER
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate)));

    driverJoystick.rightBumper().whileTrue(launchFeeder.feederOutCommand());

    driverJoystick.leftTrigger()
        .whileTrue(IntakeFactory.intakeOnlyCommand(intakeGround, intakePivot));

    driverJoystick.rightTrigger(0.1)
        .whileTrue(
            LauncherFactory.shootFeedVelocity(
                launcher, floorFeeder, launchFeeder, intakeGround, intakePivot));

    driverJoystick.leftBumper().onTrue(intakePivot.pivotStowCommand());

    driverJoystick.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    driverJoystick.b().onTrue(launcher.setHoodDegreesCommand(Constants.LauncherConstants.kHoodAngle1));
    driverJoystick.x().onTrue(launcher.setHoodDegreesCommand(Constants.LauncherConstants.kHoodAngle2));
    driverJoystick.y().onTrue(launcher.setHoodDegreesCommand(Constants.LauncherConstants.kHoodAngle3));

    driverJoystick.povUp().whileTrue(
        drivetrain.applyRequest(
            () -> point.withModuleDirection(new edu.wpi.first.math.geometry.Rotation2d(0))));
    driverJoystick.povDown().whileTrue(drivetrain.applyRequest(() -> brake));
    driverJoystick.povLeft().whileTrue(
        drivetrain.applyRequest(
            () ->
                drive.withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(-0.5 * MaxAngularRate)));
    driverJoystick.povRight().whileTrue(
        drivetrain.applyRequest(
            () ->
                drive.withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0.5 * MaxAngularRate)));

    drivetrain.registerTelemetry(logger::telemeterize);

    // OPERATOR
    operatorJoystick.leftTrigger()
        .whileTrue(IntakeFactory.intakeOnlyCommand(intakeGround, intakePivot));

    // TESTING - IntakeGround
    testingJoystick.povLeft().whileTrue(
        holdVolts(
            intakeGround,
            () -> intakeGround.setVoltage(+Constants.IntakeFloorConstants.kIntakeVolts),
            () -> intakeGround.setVoltage(0.0)));
    testingJoystick.povRight().whileTrue(
        holdVolts(
            intakeGround,
            () -> intakeGround.setVoltage(-Constants.IntakeFloorConstants.kIntakeVolts),
            () -> intakeGround.setVoltage(0.0)));

    // TESTING - IntakePivot
    testingJoystick.povUp().whileTrue(
        holdVolts(
            intakePivot,
            () -> intakePivot.setVoltage(+Constants.TestingConstants.kTestVoltsIntakePivot),
            () -> intakePivot.setVoltage(0.0)));
    testingJoystick.povDown().whileTrue(
        holdVolts(
            intakePivot,
            () -> intakePivot.setVoltage(-Constants.TestingConstants.kTestVoltsIntakePivot),
            () -> intakePivot.setVoltage(0.0)));

    // TESTING - FloorFeeder
    testingJoystick.leftTrigger(0.1).whileTrue(
        holdVolts(
            floorFeeder,
            () -> floorFeeder.setVoltage(+Constants.FloorFeederConstants.kIntakeVolts),
            () -> floorFeeder.setVoltage(0.0)));
    testingJoystick.rightTrigger(0.1).whileTrue(
        holdVolts(
            floorFeeder,
            () -> floorFeeder.setVoltage(-Constants.FloorFeederConstants.kIntakeVolts),
            () -> floorFeeder.setVoltage(0.0)));

    // TESTING - LaunchFeeder
    testingJoystick.a().whileTrue(
        holdVolts(
            launchFeeder,
            () -> launchFeeder.setVoltage(Constants.LaunchFeederConstants.kIntakeVolts),
            () -> launchFeeder.setVoltage(0.0)));
    testingJoystick.y().whileTrue(
        holdVolts(
            launchFeeder,
            () -> launchFeeder.setVoltage(-Constants.LaunchFeederConstants.kIntakeVolts),
            () -> launchFeeder.setVoltage(0.0)));

    // TESTING - Launcher
    testingJoystick.b().whileTrue(
        holdVolts(
            launcher,
            () ->
                launcher.setVoltage(
                    MathUtil.clamp(Constants.TestingConstants.kTestVoltsShooter, -12, 12)),
            () -> launcher.setVoltage(0.0)));

    // FACTORY COMMANDS
    testingJoystick.leftBumper().whileTrue(
        IntakeFactory.deployAndIntakeChainVoltage(
            intakePivot, intakeGround, floorFeeder, launchFeeder));

    testingJoystick.rightBumper().whileTrue(
        LauncherFactory.shootFeedVelocity(
            launcher, floorFeeder, launchFeeder, intakeGround, intakePivot));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}