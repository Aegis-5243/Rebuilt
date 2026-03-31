// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.roller;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class KickerSubsystem extends SubsystemBase {
    private final SparkMax kicker;
    protected final GenericEntry kickerSpeedEntry;

    public KickerSubsystem() {
        kicker = new SparkMax(Constants.KICKER, MotorType.kBrushless);

        kicker.configure(new SparkMaxConfig()
                .apply(new ClosedLoopConfig().pid(0.000065, 0, 0.07)
                        .apply(new FeedForwardConfig().kS(7))), ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        ShuffleboardTab tab = Shuffleboard.getTab("roller");
        kickerSpeedEntry = tab.add("kicker-speed-setter", 3000).getEntry();
        tab.addDouble("kicker-rpm", () -> kicker.getEncoder().getVelocity());
    }

    public void set(double speed) {
        kicker.set(speed);
    }

    public void setVelocity(AngularVelocity rpm) {
        kicker.getClosedLoopController().setSetpoint(-rpm.in(Units.RPM), ControlType.kVelocity);
    }

    public SparkMax getMotor() {
        return kicker;
    }

    @Override
    public void periodic() {}

    public Command setSpeedCommand(double speed) {
        return runEnd(() -> set(speed), () -> set(0));
    }

     public Command setVelocityCommand(AngularVelocity rpm) {
        return runEnd(() -> setVelocity(rpm), () -> set(0));
     }

     public Command runKickerCommand() {
        return setSpeedCommand(kickerSpeedEntry.getDouble(3000));
     }
}