package frc.robot.utils;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.Constants.Config;

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
}