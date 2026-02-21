// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import java.lang.reflect.Field;
import java.net.ContentHandler;
import java.util.function.DoubleSupplier;

import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.playingwithfusion.CANVenom.ControlMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.CustomMecanumDrive;
import frc.lib.CustomMecanumDriveKinematics;
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

  public SimpleMotorFeedforward flFeedforward;
  public SimpleMotorFeedforward frFeedforward;
  public SimpleMotorFeedforward blFeedforward;
  public SimpleMotorFeedforward brFeedforward;

  public CustomMecanumDrive drive;
  public MecanumDriveKinematics kinematics;
  public MecanumDrivePoseEstimator poseEstimator;

  public AHRS gyro;

  public SysIdRoutine sysId;

  public Field2d field;

  private PIDController rotController = new PIDController(0.04, 0.0, 0.0);

  private Rotation2d snapDirection = Rotation2d.kZero;

  public GenericEntry volts;

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

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

    double p = 0.3, i = 0.0, d = 0.03, f = 0.184, b = 0.0;
 // 0.195, 0, 0.01, 0.184, 0
    flMotor.setPID(p, i, d, f, b);
    frMotor.setPID(p, i, d, f, b);
    blMotor.setPID(p, i, d, f, b);
    brMotor.setPID(p, i, d, f, b);

    flFeedforward = new SimpleMotorFeedforward(Constants.DRIVE_kS, Constants.DRIVE_kV, Constants.DRIVE_kA);
    frFeedforward = new SimpleMotorFeedforward(Constants.DRIVE_kS, Constants.DRIVE_kV, Constants.DRIVE_kA);
    blFeedforward = new SimpleMotorFeedforward(Constants.DRIVE_kS, Constants.DRIVE_kV, Constants.DRIVE_kA);
    brFeedforward = new SimpleMotorFeedforward(Constants.DRIVE_kS, Constants.DRIVE_kV, Constants.DRIVE_kA);

    flMotor.setMaxSpeed(5000);
    frMotor.setMaxSpeed(5000);
    blMotor.setMaxSpeed(5000);
    brMotor.setMaxSpeed(5000);

    double maxAcceleration = Constants.DRIVE_MAX_ACCELERATION / Constants.WHEEL_DISTANCE_PER_MOTOR_REV * 60;
    flMotor.setMaxAcceleration(maxAcceleration);
    frMotor.setMaxAcceleration(maxAcceleration);
    blMotor.setMaxAcceleration(maxAcceleration);
    brMotor.setMaxAcceleration(maxAcceleration);

    double maxJerk = 10.0 * maxAcceleration;
    flMotor.setMaxJerk(maxJerk);
    frMotor.setMaxJerk(maxJerk);
    blMotor.setMaxJerk(maxJerk);
    brMotor.setMaxJerk(maxJerk);

    System.out.println(flMotor.getMaxSpeed());

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

    ControlMode controlMode = ControlMode.SpeedControl;
    /*** m/s to rpm */
    double metersPerSecondToRPM = 60 / Constants.WHEEL_DISTANCE_PER_MOTOR_REV;
    drive = new CustomMecanumDrive(
        v -> flMotor.setVoltage(flFeedforward.calculate(v)), 
        v -> blMotor.setVoltage(blFeedforward.calculate(v)),
        v -> frMotor.setVoltage(frFeedforward.calculate(v)),
        v -> brMotor.setVoltage(brFeedforward.calculate(v)));
        // v -> flMotor.set(MathUtil.clamp(0.5 * v / (Constants.DRIVE_MAX_SPEED), -1, 1)), 
        // v -> blMotor.set(MathUtil.clamp(0.5 * v / (Constants.DRIVE_MAX_SPEED), -1, 1)),
        // v -> frMotor.set(MathUtil.clamp(0.5 * v / (Constants.DRIVE_MAX_SPEED), -1, 1)),
        // v -> brMotor.set(MathUtil.clamp(0.5 * v / (Constants.DRIVE_MAX_SPEED), -1, 1)));

    // TODO: get measurements of wheels
    kinematics = new CustomMecanumDriveKinematics(
        new Translation2d(Units.Meters.of(0.2585708316), Units.Meters.of(-0.25691592)), // front left
        new Translation2d(Units.Meters.of(0.2585708316), Units.Meters.of(0.25691592)), // front right
        new Translation2d(Units.Meters.of(-0.2585708316), Units.Meters.of(-0.25691592)), // back left
        new Translation2d(Units.Meters.of(-0.2585708316), Units.Meters.of(0.25691592)), // back right
        131.0 / 133.0,
        138.0 / 161.0);

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
              .linearPosition(Units.Meters.of(-frEncoder.getDistance()))
              .linearVelocity(Units.MetersPerSecond.of(-frEncoder.getRate()));
              
          log.motor("drive-front-left")
              .voltage(Units.Volts.of(flMotor.getBusVoltage()))
              .linearPosition(Units.Meters.of(-flEncoder.getDistance()))
              .linearVelocity(Units.MetersPerSecond.of(-flEncoder.getRate()));

          log.motor("drive-back-left")
              .voltage(Units.Volts.of(blMotor.getBusVoltage()))
              .linearPosition(Units.Meters.of(-blEncoder.getDistance()))
              .linearVelocity(Units.MetersPerSecond.of(-blEncoder.getRate()));

          log.motor("drive-back-right")
              .voltage(Units.Volts.of(brMotor.getBusVoltage()))
              .linearPosition(Units.Meters.of(-brEncoder.getDistance()))
              .linearVelocity(Units.MetersPerSecond.of(-brEncoder.getRate()));
        },
        this));

    tab.addDouble("gyroYaw", gyro::getYaw);
    tab.add("flMotor", flMotor);
    tab.add("frMotor", frMotor);
    tab.add("blMotor", blMotor);
    tab.add("brMotor", brMotor);

    tab.add(drive);
    tab.addDouble("flMotorTarget", () -> flMotor.getPIDTarget());
    tab.addDouble("frMotorTarget", () -> frMotor.getPIDTarget());
    tab.addDouble("blMotorTarget", () -> blMotor.getPIDTarget());
    tab.addDouble("brMotorTarget", () -> brMotor.getPIDTarget());

    ShuffleboardTab othertab = Shuffleboard.getTab("more drivetrain");
    othertab.addDouble("flMotorHBridge", () -> flMotor.get());
    othertab.addDouble("frMotorHBridge", () -> frMotor.get());
    othertab.addDouble("blMotorHBridge", () -> blMotor.get());
    othertab.addDouble("brMotorHBridge", () -> brMotor.get());
    // ShuffleboardLayout encoderLayout =
    // tab.getLayout("encoders", "kList");

    tab.add("flEncoder", flEncoder);
    tab.add("frEncoder", frEncoder);
    tab.add("blEncoder", blEncoder);
    tab.add("brEncoder", brEncoder);

    tab.addDouble("poseX", () -> getPose().getMeasureX().in(Units.Inches));
    tab.addDouble("poseY", () -> getPose().getMeasureY().in(Units.Inches));
    tab.addDouble("poseYaw", () -> getPose().getRotation().getDegrees());
    tab.addDouble("feedforward-res", () -> flFeedforward.calculateWithVelocities(0, 2));
    volts = tab.add("drive-voltage", 0).getEntry();

    field = new Field2d();
    tab.add("Field", field);

    rotController.enableContinuousInput(-180, 180);
  }

  public double getMaxSpeed() {
    double speed = Constants.DRIVE_MAX_SPEED;
    speed += 2 * Constants.controller.getDriveBoost();

    return speed;
  }

  /**
   * @param xSpeed    meters per second (+ forward)
   * @param ySpeed    meters per second (+ left)
   * @param zRotation i don't know what unit this is (: (+ counterclockwise)
   */
  public void driveRobotCentric(double xSpeed, double ySpeed, double zRotation) {
    drive.driveCartesian(-xSpeed, ySpeed, zRotation);
  }

  /**
   * @param xSpeed    meters per second (+ forward)
   * @param ySpeed    meters per second (+ left)
   * @param zRotation i don't know what unit this is (: (+ counterclockwise)
   */
  public void driveFieldCentric(double xSpeed, double ySpeed, double zRotation) {
    // double rot = getPose().getRotation;
    Translation2d t = new Translation2d(xSpeed, ySpeed).rotateBy(getPose().getRotation().times(-1));
    xSpeed = t.getX();
    ySpeed = t.getY();
    drive.driveCartesian(-xSpeed, ySpeed, zRotation);
  }

  public Command driveFieldCentricCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier zRotation) {
    return run(
        () -> driveFieldCentric(xSpeed.getAsDouble(), ySpeed.getAsDouble(), zRotation.getAsDouble()));
  }

  public void controllerDriveFieldCentric() {
    double driveX = Constants.controller.getDriveX();
    double driveY = -Constants.controller.getDriveY();
    double driveTurn = -Constants.controller.getDriveTurn();

    driveX = Math.signum(driveX) * driveX * driveX;
    driveY = Math.signum(driveY) * driveY * driveY;
    driveTurn = Math.signum(driveTurn) * driveTurn * driveTurn;

    double speed = getMaxSpeed();

    driveX *= speed;
    driveY *= speed;
    driveTurn *= speed;

    driveFieldCentric(driveX, driveY, driveTurn);
  }

  public Command controllerDriveFieldCentricCommand = run(this::controllerDriveFieldCentric);

  public void controllerDriveFieldCentricFacingDir(Rotation2d dir) {
    double driveX = Constants.controller.getDriveX();
    double driveY = -Constants.controller.getDriveY();

    driveX = Math.signum(driveX) * driveX * driveX;
    driveY = Math.signum(driveY) * driveY * driveY;

    double speed = getMaxSpeed();

    driveX *= speed;
    driveY *= speed;

    double rotSpeed = 0;

    double angle = dir.getDegrees();

    rotController.setSetpoint(angle);
    rotSpeed = rotController.calculate(getPose().getRotation().getDegrees());

    rotSpeed = MathUtil.clamp(rotSpeed, -Constants.DRIVE_MAX_SPEED, Constants.DRIVE_MAX_SPEED);
    // }

    driveFieldCentric(driveX, driveY, rotSpeed);
  }

  public void controllerDriveFieldCentricFacingPose(double x, double y) {
    Rotation2d dir = getPose().getRotation();

    Translation2d trans = new Translation2d(x, y)
        .minus(getPose().getTranslation());
    if (trans.getNorm() > 0.1) {
      dir = trans.getAngle();
    } // Don't auto rotate when less than 10cm away from target

    controllerDriveFieldCentricFacingDir(dir);
  }

  public Command controllerDriveFieldCentricFacingPoseCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return run(() -> controllerDriveFieldCentricFacingPose(xSupplier.getAsDouble(), ySupplier.getAsDouble()));
  }

  public Command controllerDriveFieldCentricSnapCommand() {
    return startRun(() -> {
      double snapDir = 90.0;
      double angle = getPose().getRotation().getDegrees();
      angle = Math.round(angle / snapDir) * snapDir;
      snapDirection = Rotation2d.fromDegrees(angle);
    }, () -> {
      controllerDriveFieldCentricFacingDir(snapDirection);
    });
  }

  // public void driveRobotCentric(DoubleSupplier xSpeed, DoubleSupplier ySpeed,
  // DoubleSupplier zRotation) {
  // drive.driveCartesian(xSpeed.getAsDouble(), ySpeed.getAsDouble(),
  // zRotation.getAsDouble());
  // }

  public Command driveRobotCentricCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier zRotation) {
    return run(
        () -> driveRobotCentric(xSpeed.getAsDouble(), ySpeed.getAsDouble(), zRotation.getAsDouble()));
  }

  public void controllerDriveRobotCentric() {
    double driveX = Constants.controller.getDriveX();
    double driveY = -Constants.controller.getDriveY();
    double driveTurn = -Constants.controller.getDriveTurn();

    driveX = Math.signum(driveX) * driveX * driveX;
    driveY = Math.signum(driveY) * driveY * driveY;
    driveTurn = Math.signum(driveTurn) * driveTurn * driveTurn;

    double speed = getMaxSpeed();

    driveX *= speed;
    driveY *= speed;
    driveTurn *= speed;

    driveRobotCentric(driveX, driveY, driveTurn);
  }

  public Command controllerDriveRobotCentricCommand = run(this::controllerDriveRobotCentric);

  /**
   * Updates current pose using encoder positions
   */
  public void updatePose() {
    poseEstimator.update(gyro.getRotation2d(), new MecanumDriveWheelPositions(
        Units.Meters.of(flEncoder.getDistance()),
        Units.Meters.of(frEncoder.getDistance()),
        Units.Meters.of(blEncoder.getDistance()),
        Units.Meters.of(brEncoder.getDistance())));
    field.setRobotPose(getPose());
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPos() {
    poseEstimator.resetPose(new Pose2d(0.0, 0.0, Rotation2d.kZero));
  }

  public void voltageDrive() {
    flMotor.setVoltage(-volts.getDouble(0));
    frMotor.setVoltage(-volts.getDouble(0));
    blMotor.setVoltage(-volts.getDouble(0));
    brMotor.setVoltage(-volts.getDouble(0));
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

  public void stop() {
    drive.driveCartesian(0, 0, 0);
  }

}
