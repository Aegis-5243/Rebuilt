// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  public CANVenom flMotor;
  public CANVenom frMotor;
  public CANVenom blMotor;
  public CANVenom brMotor;

  public MecanumDrive drive;

  public AHRS gyro;

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

    drive = new MecanumDrive(frMotor, flMotor, brMotor, blMotor);

    gyro = new AHRS(NavXComType.kMXP_SPI);
  }

  public void drive() {
    drive.driveCartesian(Constants.controller.getLeftY(), -Constants.controller.getLeftX(), Constants.controller.getRightX());
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
