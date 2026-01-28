package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToPose extends Command {
    public Pose2d targetPose;
    public PIDController xController, yController, rotController;

    private Timer alignedTimer;

    private DriveSubsystem driveSubsystem;

    public AlignToPose(DriveSubsystem driveSubsystem, Pose2d targetPose) {
        this.driveSubsystem = driveSubsystem;
        this.targetPose = targetPose;
        xController = new PIDController(0.1, 0.0, 0.0);
        yController = new PIDController(0.1, 0.0, 0.0);
        rotController = new PIDController(0.1, 0.0, 0.0);
        rotController.enableContinuousInput(-180.0, 180.0);

        xController.setTolerance(Units.inchesToMeters(2));
        yController.setTolerance(Units.inchesToMeters(2));
        rotController.setTolerance(4);

        alignedTimer = new Timer();

        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);

        this.setName("AlignToPose " + targetPose);
    }

    @Override
    public void initialize() {
        xController.setSetpoint(targetPose.getX());
        yController.setSetpoint(targetPose.getY());
        rotController.setSetpoint(targetPose.getRotation().getDegrees());

        xController.reset();
        yController.reset();
        rotController.reset();

        alignedTimer.reset();
    }

    @Override
    public void execute() {
        Pose2d currentPose = driveSubsystem.getPose();

        double xSpeed = xController.calculate(currentPose.getX());
        double ySpeed = yController.calculate(currentPose.getY());
        double rotSpeed = rotController.calculate(currentPose.getRotation().getDegrees());

        // xSpeed = MathUtil.clamp(xSpeed, -Constants.DRIVE_MAX_SPEED, Constants.DRIVE_MAX_SPEED);
        // ySpeed = MathUtil.clamp(ySpeed, -Constants.DRIVE_MAX_SPEED, Constants.DRIVE_MAX_SPEED);
        double div = Math.sqrt(xSpeed * xSpeed + ySpeed * ySpeed);
        if (div > Constants.DRIVE_MAX_SPEED) {
            xSpeed = xSpeed / div * Constants.DRIVE_MAX_SPEED;
            ySpeed = ySpeed / div * Constants.DRIVE_MAX_SPEED;
        }

        rotSpeed = MathUtil.clamp(rotSpeed, -Constants.DRIVE_MAX_SPEED, Constants.DRIVE_MAX_SPEED);

        driveSubsystem.driveFieldCentric(xSpeed, ySpeed, rotSpeed);   
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    public boolean isAligned() {
        
        return xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint();
    }

    @Override
    public boolean isFinished() {
        if (!alignedTimer.isRunning() && isAligned()) {
            alignedTimer.restart();
        } else if (alignedTimer.isRunning() && !isAligned()) {
            alignedTimer.stop();
        }

        return isAligned() && alignedTimer.hasElapsed(0.2);
    }

}
