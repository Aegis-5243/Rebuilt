package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

public class Utilities {

    public static final Pose2d botToTurret(Pose2d botPose) {
        return botPose.transformBy(
                new Transform2d(Constants.CENTER_OF_BOT_TO_CENTER_OF_TURRET.in(Units.Meters), 0, Rotation2d.kZero));
    }

    public static final Pose2d botToCamera(Pose2d botPose, Angle turretAngle) {
        return botPose.transformBy(new Transform2d(Constants.CENTER_OF_BOT_TO_CENTER_OF_TURRET.in(Units.Meters), 0,
                Rotation2d.fromDegrees(turretAngle.in(Degrees))))
                .transformBy(new Transform2d(Constants.TURRET_RADIUS.in(Units.Meters), 0, Rotation2d.kZero));

    }

    public static final Pose2d cameraToBot(Pose2d cameraPose, Angle turretAngle, Rotation2d robotAngle) {
        // Not using MT1 rotation

        return new Pose2d(
                cameraPose.getTranslation().plus(
                        new Translation2d(Constants.TURRET_RADIUS.in(Units.Meters), 0)
                                .rotateBy(Rotation2d.fromDegrees(robotAngle.getDegrees() + (turretAngle.in(Degrees)))))
                        .plus(new Translation2d(Constants.CENTER_OF_BOT_TO_CENTER_OF_TURRET.in(Units.Meters),
                                robotAngle)),
                robotAngle);

        // cameraPose.transformBy(new Transform2d(-Constants.TURRET_RADIUS.in(Meters),
        // 0, robotAngle.plus(new Rotation2d(turretAngle))));
        // return Pose2d.kZero;
    }

}
