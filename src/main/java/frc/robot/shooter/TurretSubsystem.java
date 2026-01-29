// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
    public SparkMax turret;
    public AbsoluteEncoder turretEncoder;
    public SparkClosedLoopController pidController;

    /** Creates a new ExampleSubsystem. */
    public TurretSubsystem() {
        // TODO: config in rev hardware client
        turret = new SparkMax(Constants.TURRET_MOTOR, MotorType.kBrushless);

        turretEncoder = turret.getAbsoluteEncoder();
        turret.configure(
                new SparkMaxConfig()
                        .apply(new AbsoluteEncoderConfig().positionConversionFactor(Constants.TURRET_DEGREES_PER_REV)),
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pidController = turret.getClosedLoopController();

    }

    public double getHeading() {
        return turretEncoder.getPosition();
    }

    public void moveToHeading(double heading) {
        pidController.setSetpoint(heading, ControlType.kPosition);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
