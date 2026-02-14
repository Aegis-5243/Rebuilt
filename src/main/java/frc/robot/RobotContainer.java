// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.drive.AlignToPose;
import frc.robot.drive.DriveSubsystem;
import frc.robot.shooter.ShooterSubsystem;
// import frc.robot.shooter.HoodSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

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
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    // private final HoodSubsystem hoodSubsystem = new HoodSubsystem();
    // private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        driveSubsystem.setDefaultCommand(driveSubsystem.controllerDriveRobotCentricCommand);
        // shooterSubsystem.setDefaultCommand(shooterSubsystem.run(() -> {
        //     shooterSubsystem.setDutyCycle(0);
        // }));
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
        // new JoystickButton(Constants.controller,
        // XboxController.Button.kA.value).whileTrue(driveSubsystem.sysId.quasistatic(Direction.kForward));
        // new JoystickButton(Constants.controller,
        // XboxController.Button.kB.value).whileTrue(driveSubsystem.sysId.quasistatic(Direction.kReverse));
        // new JoystickButton(Constants.controller,
        // XboxController.Button.kX.value).whileTrue(driveSubsystem.sysId.dynamic(Direction.kForward));
        // new JoystickButton(Constants.controller,
        // XboxController.Button.kY.value).whileTrue(driveSubsystem.sysId.dynamic(Direction.kReverse));

        // Drive field centric
        new Trigger(() -> Constants.controller.getDriveFieldCentricMode())
        .whileTrue(driveSubsystem.controllerDriveFieldCentricCommand);

        // Drive field centric facing origin
        new Trigger(() ->
        Constants.controller.getDriveFieldCentricFacingOriginMode())
        .whileTrue(driveSubsystem.controllerDriveFieldCentricFacingPoseCommand(() ->
        0, () -> 0));

        // Drive field centric snapping
        new Trigger(() -> Constants.controller.getDriveFieldCentricSnappingMode())
        .whileTrue(driveSubsystem.controllerDriveFieldCentricSnapCommand());

        // Reset pose to origin
        new Trigger(() -> Constants.controller.getResetPoseButton())
        .onTrue(Commands.runOnce(driveSubsystem::resetPos));

        // Align to origin pose with deceleration
        new Trigger(() -> Constants.controller.getAlignToOriginPoseButton())
        .whileTrue(new AlignToPose(driveSubsystem, new Pose2d(0, 0,
        Rotation2d.kZero)));

        // new Trigger(() -> Constants.controller.getShoot()).whileTrue(shooterSubsystem.runEnd(() -> {
        //     shooterSubsystem.setDutyCycle(.75);
        
        //     System.out.println("duty cycle");
        // }, () -> shooterSubsystem.setDutyCycle(0)));
        
        // new Trigger(() -> Constants.controller.getAlignToOriginPoseButton()).whileTrue(shooterSubsystem.runEnd(() -> {
        //     shooterSubsystem.setVelocity(Units.RPM.of(6000));
        
        //     System.out.println("rpm");
        // }, () -> shooterSubsystem.setDutyCycle(0)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        // return driveSubsystem.driveRobotCentricCommand(() -> 1.0, () -> 0.0, () ->
        // 0.0)
        // .withDeadline(Commands.waitSeconds(1));
        return new InstantCommand();
        // return Autos.exampleAuto(m_driveSubsystem);
    }
}
