package frc.robot.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class AlignToPoseAccelerative extends Command {
    public Pose2d targetPose;
    public PIDController xController, yController, rotController;

    private Timer alignedTimer;

    private DriveSubsystem driveSubsystem;

    public AlignToPoseAccelerative(DriveSubsystem driveSubsystem, Pose2d targetPose) {
        this.driveSubsystem = driveSubsystem;
        this.targetPose = targetPose;
        xController = new PIDController(2, 0.0, 0.0);
        yController = new PIDController(2, 0.0, 0.0);
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
        
        double xError = xController.getError();
        double yError = yController.getError();
        double dist = Math.sqrt(xError * xError + yError * yError);

        double deceleration = Constants.DRIVE_MAX_ACCELERATION*0.5;

        double maxSpeed = Math.sqrt(2 * deceleration * dist); // add a small constant to prevent stalling
        maxSpeed = MathUtil.clamp(maxSpeed, 0.2, Constants.DRIVE_MAX_SPEED);

        SmartDashboard.putNumber("align_rawXSpeed", xSpeed);
        SmartDashboard.putNumber("align_rawYSpeed", ySpeed);
        
        double div = Math.sqrt(xSpeed * xSpeed + ySpeed * ySpeed);
        if (div > maxSpeed) {
            xSpeed = xSpeed / div * maxSpeed;
            ySpeed = ySpeed / div * maxSpeed;
        }

        rotSpeed = MathUtil.clamp(rotSpeed, -Constants.DRIVE_MAX_SPEED, Constants.DRIVE_MAX_SPEED);

        driveSubsystem.driveFieldCentric(xSpeed, ySpeed, rotSpeed);

        SmartDashboard.putNumber("align_clampedXSpeed", xSpeed);
        SmartDashboard.putNumber("align_clampedYSpeed", ySpeed);
        SmartDashboard.putNumber("align_rotSpeed", rotSpeed);
        SmartDashboard.putNumber("align_dist", dist);
        SmartDashboard.putNumber("align_maxSpeed", maxSpeed);
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
