// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private SparkMax intake;
    private TimeOfFlight intakeTof;
    public int intakedBalls;

    /** Creates a new ExampleSubsystem. */
    public IntakeSubsystem() {
        // they are all following each other for now.
        intake = new SparkMax(Constants.INTAKE, MotorType.kBrushless);
        intakeTof = new TimeOfFlight(34);
        intakedBalls = 0;

        new Trigger(() -> {return intakeTof.getRange() < 265;}).onTrue(new InstantCommand(() -> intakedBalls++));
    }
    
    @Override
    public void periodic() {

        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    private void setIntake(double speed) {
        intake.set(speed);
    }

    public Command setIntakeCommand(double speed) {
        return runEnd(() -> setIntake(speed), () -> setIntake(0));
    }
}
