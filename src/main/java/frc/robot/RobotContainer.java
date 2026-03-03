// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.camera.CameraSubsystem;
import frc.robot.drive.DriveSubsystem;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.shooter.HoodSubsystem;
import frc.robot.shooter.RollerSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.shooter.TurretSubsystem;
import frc.robot.utils.Kinematics;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final TurretSubsystem turretSubsystem = new TurretSubsystem();
    private final DriveSubsystem driveSubsystem = new DriveSubsystem(turretSubsystem);
    private final HoodSubsystem hoodSubsystem = new HoodSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final RollerSubsystem rollerSubsystem = new RollerSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final CameraSubsystem cameraSubsystem = new CameraSubsystem(driveSubsystem);
    // private final CameraSubsystem cameraSubsystem = new
    // CameraSubsystem(driveSubsystem::getPose,
    // () -> driveSubsystem.gyro.getAngle(), () ->
    // Units.DegreesPerSecond.of(driveSubsystem.gyro.getRate()));

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        driveSubsystem.setDefaultCommand(driveSubsystem.controllerDriveRobotCentricCommand);
        shooterSubsystem.setDefaultCommand(shooterSubsystem.run(() -> {
            shooterSubsystem.setDutyCycle(0);
        }).withName("shooterDefault"));
        rollerSubsystem.setDefaultCommand(rollerSubsystem.run(() -> {
            rollerSubsystem.set(0);
        }).withName("rollerDefault"));
        intakeSubsystem
                .setDefaultCommand(intakeSubsystem.run(() -> intakeSubsystem.intake.set(0)).withName("intakeDefault"));
        // hoodSubsystem.setDefaultCommand(hoodSubsystem.run(() ->
        // hoodSubsystem.setPos(MathUtil
        // .clamp(hoodSubsystem.primaryHoodServo.get() +
        // Constants.controller.getHoodDisplacement() * 0.05, 0,
        // 1)))
        // .withName("hoodDefault"));
        hoodSubsystem.setDefaultCommand(hoodSubsystem.run(() -> hoodSubsystem.setPos(MathUtil
                .clamp(hoodSubsystem.hoodSetpoint.getDouble(0), 0,
                        1)))
                .withName("hoodDefault"));
        turretSubsystem.setDefaultCommand(turretSubsystem.run(() -> {
            turretSubsystem.setPower(Constants.controller.getTurretDisplacement() * 0.2);
        }).withName("turretDefault"));
        CommandScheduler.getInstance().registerSubsystem(cameraSubsystem);

        driveSubsystem.setLimelightPoseSupplier(cameraSubsystem::getPose);
        driveSubsystem.setLimelightTimestampSupplier(() -> cameraSubsystem.timestamp);
        driveSubsystem.setTurretAngleSupplier(() -> Units.Degrees.of(turretSubsystem.getHeading()));
        driveSubsystem.setLimelightUpdateSupplier(() -> !cameraSubsystem.rejectUpdate);

        Shuffleboard.getTab("Teleoperated").add("CS", CommandScheduler.getInstance());

        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // temp binds for drivetrain characterization
        // new Trigger(() -> Constants.controller.getDriveFieldCentricMode())
        // .whileTrue(driveSubsystem.sysId.quasistatic(Direction.kForward));
        // new Trigger(() ->
        // Constants.controller.getDriveFieldCentricFacingOriginMode())
        // .whileTrue(driveSubsystem.sysId.quasistatic(Direction.kReverse));
        // new Trigger(() -> Constants.controller.getDriveFieldCentricSnappingMode())
        // .whileTrue(driveSubsystem.sysId.dynamic(Direction.kForward));
        // new Trigger(() -> Constants.controller.getResetPoseButton())
        // .whileTrue(driveSubsystem.sysId.dynamic(Direction.kReverse));
        new Trigger(() -> Constants.controller.getResetPoseButton())
                .onTrue(Commands.runOnce(() -> driveSubsystem.resetPos()));
        // .whileTrue(driveSubsystem.run(() -> driveSubsystem.voltageDrive()));

        // Drive field centric
        // new Trigger(() -> Constants.controller.getDriveFieldCentricMode())
        // .whileTrue(driveSubsystem.controllerDriveFieldCentricCommand);

        // // Drive field centric facing origin
        // new Trigger(() ->
        // Constants.controller.getDriveFieldCentricFacingOriginMode())
        // .whileTrue(driveSubsystem.controllerDriveFieldCentricFacingPoseCommand(() ->
        // 0, () -> 0));

        // // Drive field centric snapping
        // new Trigger(() -> Constants.controller.getDriveFieldCentricSnappingMode())
        // .whileTrue(driveSubsystem.controllerDriveFieldCentricSnapCommand());

        // // Reset pose to origin
        new Trigger(() -> Constants.controller.getResetPoseButton())
                .onTrue(Commands.runOnce(driveSubsystem::setPoseToCam));

        // // Align to origin pose with deceleration
        // new Trigger(() -> Constants.controller.getAlignToOriginPoseButton())
        // .whileTrue(new AlignToPose(driveSubsystem, new Pose2d(0, 0,
        // Rotation2d.kZero)));

        // new Trigger(() ->
        // Constants.controller.getResetPoseButton()).whileTrue(turretSubsystem.run(()
        // -> {
        // double theta = cameraSubsystem.getThetaDiff();
        // turretSubsystem.setPower(-(theta / 180 * .5));
        // System.out.println(theta);
        // }));

        new Trigger(() -> Constants.controller.getShoot()).whileTrue(shooterSubsystem.runEnd(() -> {
            shooterSubsystem.setVelocity(Units.RPM.of(shooterSubsystem.targetRPM.getDouble(6000)));

        }, () -> shooterSubsystem.setDutyCycle(0)));

        new Trigger(() -> Constants.controller.getRoller()).whileTrue(
                rollerSubsystem.runEnd(() -> {
                    rollerSubsystem.set(.5, rollerSubsystem.kickerSpeed.getDouble(0.5));
                }, () -> rollerSubsystem.set(0)));

        new Trigger(() -> Constants.controller.getIntake()).whileTrue(
                intakeSubsystem.runEnd(() -> intakeSubsystem.intake.set(.9), () -> intakeSubsystem.intake.set(0)));

        new Trigger(() -> Constants.controller.getDriveFieldCentricFacingHubMode()).whileTrue(faceHubCommand);
    }

    public Command faceHubCommand = Commands.run(() -> {
        double angle = Kinematics
                .getHubTransform2d(driveSubsystem.botToTurret(driveSubsystem.getFutureRobotPose2d(0.0004)))
                .getRotation().getDegrees();
        angle = MathUtil.clamp(angle, -90, 90);

        turretSubsystem.setTarget(angle);
    }, turretSubsystem);

    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        // return driveSubsystem.driveRobotCentricCommand(() -> 1.0, () -> 0.0, () ->
        // 0.0)
        // .withDeadline(Commands.waitSeconds(1));
        return new InstantCommand();
        // return Autos.exampleAuto(m_driveSubsystem);
    }
}
