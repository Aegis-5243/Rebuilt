package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ProController implements DriveController {
    private XboxController controller;

    public ProController(int port) {
        controller = new XboxController(port);
        ShuffleboardTab tab = Shuffleboard.getTab("Controller");

        tab.addDouble("driveX", () -> getDriveX());
        tab.addDouble("driveY", () -> getDriveY());
        tab.addDouble("driveTurn", () -> getTurretDisplacement());
    }
    
    @Override
    public double getDriveX() {
        return -controller.getLeftY();
    }

    @Override
    public double getDriveY() {
        return controller.getLeftX();
    }

    @Override
    public double getDriveTurn() {
        return controller.getRightX();
    }

    @Override
    public double getDriveBoost() {
        return MathUtil.clamp(2 * controller.getLeftTriggerAxis() - 1.0, 0.0, 1.0);
    }

    @Override
    public boolean getDriveFieldCentricMode() {
        return controller.getLeftStickButton();
    }

    @Override
    public boolean getDriveFieldCentricFacingHubMode() {
        return controller.getRightStickButton();
    }

    @Override
    public boolean getDriveFieldCentricSnappingMode() {
        // return controller.getRightBumperButton();
        return controller.getXButton();
    }

    @Override
    public boolean getResetPoseButton() {
        return controller.getBackButton();
    }

    @Override
    public boolean getAlignToOriginPoseButton() {
        return controller.getBButton();
    }

    @Override
    public boolean getShoot() {
        // return controller.getAButton();
        return controller.getRightTriggerAxis() > 0.6;// || controller.getBButton();
    }

    @Override
    public boolean getIntake() {
        return controller.getLeftBumperButton();// || controller.getBButton();
        // return controller.getXButton();
    }

    @Override
    public boolean getRoller() {
        // return controller.getYButton();
        return controller.getRightBumperButton();// || controller.getBButton();
    }

    @Override
    public double getHoodDisplacement() {
        double res = controller.getPOV();

        
        if (res == 0) 
            return 1;
        else if (res == 180)
            return -1;
        return 0;
    }

    public double getTurretDisplacement() {
        double res = controller.getPOV();

        if (res == 90) 
            return -1;
        else if (res == 270)
            return 1;
        return 0;
    }

    @Override
    public boolean allShoot() {
        return controller.getAButton();
    }

    @Override
    public boolean climbUp() {
        return controller.getYButton();
    }

    @Override
    public boolean climbDown() {
        return controller.getBButton();
    }
}
