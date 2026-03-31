// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.roller;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Aggregator subsystem that exposes the old RollerSubsystem API but delegates
 * motor control to HopperSubsystem (roller) and KickerSubsystem (kicker).
 */
public class RollerSubsystem extends SubsystemBase {
    public final HopperSubsystem hopper;
    public final KickerSubsystem kicker;

    public GenericEntry kickerSpeedEntry;

    public RollerSubsystem() {
        hopper = new HopperSubsystem();
        kicker = new KickerSubsystem();

        kickerSpeedEntry = kicker.kickerSpeedEntry;
    }

    public void set(double speed) {
        double rollerSpeed = speed;
        if (Constants.controller.getReverseRollers()) {
            rollerSpeed = -0.5;
        }
        hopper.set(rollerSpeed);
        kicker.set(speed);
    }

    public void set(double rollerSpeed, AngularVelocity kickerSpeed) {
        if (Constants.controller.getReverseRollers()) {
            rollerSpeed = -0.5;
        }
        hopper.set(rollerSpeed);
        kicker.setVelocity(kickerSpeed);
    }

    public Command runRollers() {
        return new ParallelCommandGroup(
            hopper.runWithStallDetectionCommand(),
            kicker.setSpeedCommand(kickerSpeedEntry.getDouble(3000))
        );
    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}

}
