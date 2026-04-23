// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.WrappingDutyCycleEncoder;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
    public SparkMax turret;
    public RelativeEncoder turretEncoder;
    public ColorSensorV3 colorSensor;
    private WrappingDutyCycleEncoder temp;
    public DigitalInput limitSwitch;

    public SlewRateLimiter rateLimiter;

    /** Creates a new ExampleSubsystem. */
    public TurretSubsystem() {
        // TODO: config in rev hardware client
        turret = new SparkMax(Constants.TURRET_MOTOR, MotorType.kBrushless);

        double tP = 0.016;
        double tI = 0.00;
        double tD = 0.005;

        turretEncoder = turret.getEncoder();

        turret.configure(
                new SparkMaxConfig()
                        .apply(new EncoderConfig().positionConversionFactor(Constants.TURRET_DEGREES_PER_REV))
                        .apply(new SparkMaxConfig().idleMode(IdleMode.kBrake))
                        .apply(new ClosedLoopConfig()
                                .pid(tP, tI, tD)
                                .maxOutput(0.5)
                                .minOutput(-0.5))
                        .apply(new SoftLimitConfig()
                                .forwardSoftLimit(Constants.TURRET_MAX_ANGLE.in(Degrees)).forwardSoftLimitEnabled(true)
                                .reverseSoftLimit(Constants.TURRET_MIN_ANGLE.in(Degrees)).reverseSoftLimitEnabled(true)),
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        colorSensor = new ColorSensorV3(I2C.Port.kMXP);

        temp = new WrappingDutyCycleEncoder(9, turretEncoder::getVelocity, 0, true);
        limitSwitch = new DigitalInput(8);
        // encoder.
        Shuffleboard.getTab("turret").addDouble("turret-encoder", () -> turretEncoder.getPosition());
        Shuffleboard.getTab("turret").addDouble("turret-encoder-vel", turretEncoder::getVelocity);
        if (DriverStation.isTest()) {
            Shuffleboard.getTab("turret").addDouble("turret-encoder2", () -> temp.get());
            Shuffleboard.getTab("turret").addBoolean("turretEncoderConnected", () -> temp.isConnected());
            Shuffleboard.getTab("color").addDouble("color-blue", colorSensor::getBlue);
            Shuffleboard.getTab("color").addDouble("color-green", colorSensor::getGreen);
            Shuffleboard.getTab("color").addDouble("color-red", colorSensor::getRed);
            Shuffleboard.getTab("color").addBoolean("color-connected", colorSensor::isConnected);
            Shuffleboard.getTab("turret").addBoolean("turret-limit", limitSwitch::get);
        }

        Shuffleboard.getTab("turret").add("Reset turret to forward",
                runOnce(() -> turretEncoder.setPosition(0)).ignoringDisable(true));
        Shuffleboard.getTab("turret").add("Reset turret to right",
                runOnce(() -> turretEncoder.setPosition(-90)).ignoringDisable(true));

        rateLimiter = new SlewRateLimiter(20);

        // turretEncoder.setPosition(-90); // ASSUME RIGHTWARD TURRET ON POWER CYCLE
    }

    public double getHeading() {
        return turretEncoder.getPosition();
    }

    public void setTarget(double pos) {
        turret.getClosedLoopController().setSetpoint(pos, ControlType.kPosition);
    }

    public boolean atSetpoint() {
        return turret.getClosedLoopController().isAtSetpoint();
    }

    public void setPower(double power) {
        if (power < 0 && turretEncoder.getPosition() > -90)
            turret.set(power);
        else if (power > 0 && turretEncoder.getPosition() < 90)
            turret.set(power);
        else
            turret.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        temp.update();
        // if (limitSwitch.get()) {
        //     turretEncoder.setPosition(-90);
        // }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
