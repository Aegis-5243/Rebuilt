// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.roller;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class HopperSubsystem extends SubsystemBase {
    private final SparkMax roller;
    private final Trigger rollerStallTrigger;

    public HopperSubsystem() {
        roller = new SparkMax(Constants.ROLLER, MotorType.kBrushless);
        RelativeEncoder rollerEncoder = roller.getEncoder();

        ShuffleboardTab tab = Shuffleboard.getTab("roller");
        tab.addDouble("rollerCurrent", roller::getOutputCurrent);
        tab.addDouble("rollerVelocity", rollerEncoder::getVelocity);

        rollerStallTrigger = new Trigger(() -> {
            double current = roller.getOutputCurrent();
            double velocity = rollerEncoder.getVelocity();
            return (current > 60 && Math.abs(velocity) < 1000);
        });

        tab.addBoolean("rollerStalled", rollerStallTrigger::getAsBoolean);
    }

    public void set(double speed) {
        double rollerSpeed = speed;
        if (Constants.controller.getReverseRollers()) {
            rollerSpeed = -0.5;
        }
        roller.set(rollerSpeed);
    }

    public SparkMax getMotor() {
        return roller;
    }

    @Override
    public void periodic() {
    }

    public Command setSpeedCommand(double speed) {
        return runEnd(() -> set(speed), () -> set(0));
    }

    public Command runWithStallDetectionCommand() {
        double DebounceTime = 0.1;
        double reverseTime = 0.3;
        return new RepeatCommand(
                setSpeedCommand(Constants.ROLLER_SPEED)
                        .until(rollerStallTrigger.debounce(DebounceTime))
                        .andThen(setSpeedCommand(Constants.ROLLER_SPEED_REVERSE))
                        .withTimeout(reverseTime));
    }
}
