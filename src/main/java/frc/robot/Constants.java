// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.XboxController;

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

    public static final int[] FL_ENCODER = { 2, 3 };
    public static final int[] FR_ENCODER = { 6, 7 };
    public static final int[] BL_ENCODER = { 0, 1 };
    public static final int[] BR_ENCODER = { 4, 5 };

    public static final int ENCODER_CYCLES_PER_REV = 2048;

    public static final Distance WHEEL_DIAMETER = Units.Inches.of(6);
    public static final double WHEEL_DISTANCE_PER_PULSE = Math.PI * Constants.WHEEL_DIAMETER.in(Units.Meters) / Constants.ENCODER_CYCLES_PER_REV;

    public static XboxController controller = new XboxController(0);

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }
}
