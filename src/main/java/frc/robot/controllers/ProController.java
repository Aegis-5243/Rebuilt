package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;

public class ProController implements DriveController {
    private XboxController controller;

    public ProController(int port) {
        controller = new XboxController(port);
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
    public boolean getDriveFieldCentricFacingOriginMode() {
        return controller.getRightStickButton();
    }

    @Override
    public boolean getDriveFieldCentricSnappingMode() {
        return controller.getRightBumperButton();
    }

    @Override
    public boolean getResetPoseButton() {
        return controller.getBackButton();
    }

    @Override
    public boolean getAlignToOriginPoseButton() {
        return controller.getBButton();
    }
}
