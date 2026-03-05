package frc.robot.utils;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;

public class Utilites {
    public static Config distanceToConfig(Distance distance) {
        Distance closest = Units.Meters.of(100000000);
        for (Distance item : Constants.shooter_configs.keySet()) {
            if (distance.minus(item).abs(Units.Meters) < distance.minus(closest).abs(Units.Meters)) {
                closest = item;
            }
        }

        return Constants.shooter_configs.get(closest);
    }
    
    public static class Config {
        public double shooter_rpm;
        public double servo_pos;
        public double kicker_rpm;

        public Config(double shooter_rpm, double servo_pos, double kicker_rpm) {
            this.shooter_rpm = shooter_rpm;
            this.servo_pos = servo_pos;
            this.kicker_rpm = kicker_rpm;
        }
    }
}