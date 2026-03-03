// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RollerSubsystem extends SubsystemBase {
    public SparkMax roller;
    public SparkMax kicker;
    public GenericEntry kickerSpeed;

    /** Creates a new ExampleSubsystem. */
    public RollerSubsystem() {
        // they are all following each other for now.
        roller = new SparkMax(Constants.ROLLER, MotorType.kBrushless);
        kicker = new SparkMax(Constants.KICKER, MotorType.kBrushless);

        kickerSpeed = Shuffleboard.getTab("roller").add("kicker-speed-setter", 0.5).getEntry();
    }

    public void set(double speed) {
        roller.set(speed);
        kicker.set(speed);
    }

    public void set(double rollerSpeed, double kickerSpeed) {
        roller.set(rollerSpeed);
        kicker.set(kickerSpeed);
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
