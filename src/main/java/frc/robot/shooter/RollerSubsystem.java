// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RollerSubsystem extends SubsystemBase {
    public SparkMax roller;
    public SparkMax kicker;
    public GenericEntry kickerSpeed;

    /** Creates a new ExampleSubsystem. */
    public RollerSubsystem() {
        roller = new SparkMax(Constants.ROLLER, MotorType.kBrushless);
        kicker = new SparkMax(Constants.KICKER, MotorType.kBrushless);

        kicker.configure(new SparkMaxConfig().apply(new ClosedLoopConfig().pid(0.0001, 0, 0).apply(new FeedForwardConfig().kS(6))), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        kickerSpeed = Shuffleboard.getTab("roller").add("kicker-speed-setter", 3000).getEntry();
        Shuffleboard.getTab("roller").addDouble("kicker-rpm", () -> kicker.getEncoder().getVelocity());


    }

    public void set(double speed) {
        roller.set(speed);
        kicker.set(speed);
    }

    public void set(double rollerSpeed, AngularVelocity kickerSpeed) {
        roller.set(rollerSpeed);
        kicker.getClosedLoopController().setSetpoint(-kickerSpeed.in(Units.RPM), ControlType.kVelocity);
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
