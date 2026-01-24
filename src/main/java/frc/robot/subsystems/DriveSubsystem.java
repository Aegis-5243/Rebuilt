// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

    public CANVenom flMotor;
    public CANVenom frMotor;
    public CANVenom blMotor;
    public CANVenom brMotor;

    public Encoder flEncoder;
    public Encoder frEncoder;
    public Encoder blEncoder;
    public Encoder brEncoder;

    public MecanumDrive drive;
    public MecanumDriveKinematics kinematics;
    public MecanumDrivePoseEstimator poseEstimator;

    public AHRS gyro;

    public SysIdRoutine sysId;

    /** Creates a new ExampleSubsystem. */
    public DriveSubsystem() {

        flMotor = new CANVenom(Constants.FL_MOTOR);
        frMotor = new CANVenom(Constants.FR_MOTOR);
        blMotor = new CANVenom(Constants.BL_MOTOR);
        brMotor = new CANVenom(Constants.BR_MOTOR);

        flMotor.setInverted(true);
        frMotor.setInverted(false);
        blMotor.setInverted(true);
        brMotor.setInverted(false);

        flMotor.setBrakeCoastMode(BrakeCoastMode.Brake);
        frMotor.setBrakeCoastMode(BrakeCoastMode.Brake);
        blMotor.setBrakeCoastMode(BrakeCoastMode.Brake);
        brMotor.setBrakeCoastMode(BrakeCoastMode.Brake);

        flEncoder = new Encoder(Constants.FL_ENCODER[0], Constants.FL_ENCODER[1], true);
        frEncoder = new Encoder(Constants.FR_ENCODER[0], Constants.FR_ENCODER[1], false);
        blEncoder = new Encoder(Constants.BL_ENCODER[0], Constants.BL_ENCODER[1], true);
        brEncoder = new Encoder(Constants.BR_ENCODER[0], Constants.BR_ENCODER[1], false);

        // this makes the encoder return the wheels linear distance and velocity in
        // meters and meters per second
        flEncoder.setDistancePerPulse(Constants.WHEEL_DISTANCE_PER_PULSE);
        frEncoder.setDistancePerPulse(Constants.WHEEL_DISTANCE_PER_PULSE);
        blEncoder.setDistancePerPulse(Constants.WHEEL_DISTANCE_PER_PULSE);
        brEncoder.setDistancePerPulse(Constants.WHEEL_DISTANCE_PER_PULSE);
        
        gyro = new AHRS(NavXComType.kMXP_SPI);
        gyro.reset();

        drive = new MecanumDrive(flMotor, blMotor, frMotor, brMotor);

        // TODO: get measurements of wheels
        kinematics = new MecanumDriveKinematics(
                new Translation2d(Units.Meters.of(0), Units.Meters.of(0)), // front left
                new Translation2d(Units.Meters.of(0), Units.Meters.of(0)), // front right
                new Translation2d(Units.Meters.of(0), Units.Meters.of(0)), // back left
                new Translation2d(Units.Meters.of(0), Units.Meters.of(0))); // back right

        poseEstimator = new MecanumDrivePoseEstimator(kinematics, gyro.getRotation2d(),
                new MecanumDriveWheelPositions(), Pose2d.kZero);

        this.sysId = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(
                voltage -> {
                    flMotor.setVoltage(voltage.magnitude());
                    frMotor.setVoltage(voltage.magnitude());
                    blMotor.setVoltage(voltage.magnitude());
                    brMotor.setVoltage(voltage.magnitude());
                },
                log -> {
                    log.motor("drive-front-right")
                            .voltage(Units.Volts.of(frMotor.getBusVoltage()))
                            .linearPosition(Units.Meters.of(frEncoder.getDistance()))
                            .linearVelocity(Units.MetersPerSecond.of(frEncoder.getRate()));

                    log.motor("drive-front-left")
                            .voltage(Units.Volts.of(flMotor.getBusVoltage()))
                            .linearPosition(Units.Meters.of(flEncoder.getDistance()))
                            .linearVelocity(Units.MetersPerSecond.of(flEncoder.getRate()));

                    log.motor("drive-back-left")
                            .voltage(Units.Volts.of(blMotor.getBusVoltage()))
                            .linearPosition(Units.Meters.of(blEncoder.getDistance()))
                            .linearVelocity(Units.MetersPerSecond.of(blEncoder.getRate()));

                    log.motor("drive-back-right")
                            .voltage(Units.Volts.of(brMotor.getBusVoltage()))
                            .linearPosition(Units.Meters.of(brEncoder.getDistance()))
                            .linearVelocity(Units.MetersPerSecond.of(brEncoder.getRate()));
                },
                this));

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        tab.addDouble("gyroYaw", gyro::getYaw);
        tab.add("flMotor", flMotor);
        tab.add("frMotor", frMotor);
        tab.add("blMotor", blMotor);
        tab.add("brMotor", brMotor);
        // ShuffleboardLayout encoderLayout =
        // tab.getLayout("encoders", "kList");

        tab.add("flEncoder", flEncoder);
        tab.add("frEncoder", frEncoder);
        tab.add("blEncoder", blEncoder);
        tab.add("brEncoder", brEncoder);
    }

    /**
     * Drives the robot using controller input.
     */
    public void drive() {

        double leftY = Constants.controller.getLeftY();
        double leftX = -Constants.controller.getLeftX();
        double rightX = -Constants.controller.getRightX();

        leftY = Math.signum(leftY) * leftY * leftY;
        leftX = Math.signum(leftX) * leftX * leftX;
        rightX = Math.signum(rightX) * rightX * rightX;

        leftY *= 0.75;
        leftX *= 0.75;
        rightX *= 0.75;

        drive.driveCartesian(leftY, leftX, rightX);
    }

    /**
     * Updates current pose using encoder positions
     */
    public void updatePose() {
        poseEstimator.update(gyro.getRotation2d(), new MecanumDriveWheelPositions(
                Units.Meters.of(flEncoder.getDistance()),
                Units.Meters.of(frEncoder.getDistance()),
                Units.Meters.of(blEncoder.getDistance()),
                Units.Meters.of(brEncoder.getDistance())));
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updatePose();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
