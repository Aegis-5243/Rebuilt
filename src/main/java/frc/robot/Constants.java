// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.controllers.DriveController;
import frc.robot.controllers.ProController;
import frc.robot.utils.Utilites.Config;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int FL_MOTOR = 13;
    public static final int FR_MOTOR = 14;
    public static final int BL_MOTOR = 11;
    public static final int BR_MOTOR = 12;

    public static final int[] FL_ENCODER = { 4, 5 };
    public static final int[] FR_ENCODER = { 2, 3 };
    public static final int[] BL_ENCODER = { 6, 7 };
    public static final int[] BR_ENCODER = { 0, 1 };

    public static final double DRIVE_kS = 0.725;
    public static final double DRIVE_kV = 2.58;
    public static final double DRIVE_kA = 0.59;

    public static final int ENCODER_CYCLES_PER_REV = 2048;

    public static final double DRIVE_GEARBOX_RATIO = 9.13;

    public static final Distance WHEEL_DIAMETER = Units.Inches.of(6);
    /*** meters per revolution */
    public static final double WHEEL_DISTANCE_PER_MOTOR_REV = Math.PI * WHEEL_DIAMETER.in(Units.Meters)
            / DRIVE_GEARBOX_RATIO;
    /*** meters per encoder pulse */
    public static final double WHEEL_DISTANCE_PER_PULSE = Math.PI * Constants.WHEEL_DIAMETER.in(Units.Meters)
            / Constants.ENCODER_CYCLES_PER_REV;

    /*** Meters per second */
    public static final double DRIVE_MAX_SPEED = 2.0;
    /*** Meters per second squared */
    public static final double DRIVE_MAX_ACCELERATION = 10.0;

    public static final int TURRET_MOTOR = 41;
    /*** Degrees per revolution */
    public static final double TURRET_DEGREES_PER_REV = 7.5; /* original 7.875 */
    /*** Distance between center of turret and limelight lens */
    public static final Distance TURRET_RADIUS = Units.Inches.of(7.5);
    public static final Distance CENTER_OF_BOT_TO_CENTER_OF_TURRET = Units.Inches.of(8.5);
    public static final String TURRET_LIMELIGHT = "";

    // limelight hight 19.25
    // pitch 28.725 degress

    public static final int PRIMARY_SHOOTER = 31;
    public static final int SECONDARY_SHOOTER = 32;

    public static final double SHOOTER_kP = 0.95;
    public static final double SHOOTER_kI = 0;
    public static final double SHOOTER_kD = 0;
    public static final double SHOOTER_kF = 10;

    public static final double SHOOTER_kA = 0;
    public static final double SHOOTER_kS = 0;
    public static final double SHOOTER_kV = 0;

    public static final int ROLLER = 22;
    public static final int INTAKE = 21;
    public static final int KICKER = 23;

    public static final Map<Distance, Config> shooter_configs = Map.of(
            Units.Meters.of(1.5), new Config(3750, 0, 3000),
            Units.Meters.of(1.8), new Config(4175, 0.1, 3000),
            Units.Meters.of(2.25), new Config(5750, 0.2, 3500),
            Units.Meters.of(2.5), new Config(6000, 0.25, 3750),
            Units.Meters.of(2.946), new Config(6000, 0.4, 4000));
    
    /* TODO */
    public static final int CLIMB_MOTOR = 43;
    
    /* rotations to inches
     * 1:162 gear and gearbox ratio
     * 35 chain: 3/8" pitch
     * 15 tooth interface
     */
    /** rotations to inches */
    public static final double CLIMB_POSISION_CONVERSION_FACTOR = 3.0/8.0 * 15.0 / 162.0;
    /** RPM to inches per second */
    public static final double CLIMB_VELOCITY_CONVERSION_FACTOR = CLIMB_POSISION_CONVERSION_FACTOR / 60.0;

    public static DriveController controller = new ProController(0);

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }
}
