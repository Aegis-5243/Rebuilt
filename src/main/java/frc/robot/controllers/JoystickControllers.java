package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;

public class JoystickControllers implements DriveController {
    public Joystick primaryController;
    public Joystick secondaryController;

    public JoystickControllers(int primaryPort, int secondaryPort) {
        primaryController = new Joystick(primaryPort);
        secondaryController = new Joystick(secondaryPort);
    }

    @Override
    public double getDriveX() {
        return -primaryController.getY();
    }

    @Override
    public double getDriveY() {
        return primaryController.getX();
    }

    @Override
    public double getDriveTurn() {
        return secondaryController.getX();
    }

    @Override
    public double getDriveBoost() {
        return MathUtil.clamp(secondaryController.getThrottle() + 1.0 / 2.0, 0.0, 1.0);
    }

    @Override
    public boolean getDriveFieldCentricMode() {
        return primaryController.getTrigger();
    }

    @Override
    public boolean getDriveFieldCentricFacingOriginMode() {
        return false;
    }

    @Override
    public boolean getDriveFieldCentricSnappingMode() {
        return false;
    }

    @Override
    public boolean getResetPoseButton() {
        return primaryController.getRawButton(5);
    }

    @Override
    public boolean getAlignToOriginPoseButton() {
        return false;
    }

    @Override
    public boolean getShoot() {
        return primaryController.getTrigger();
    }

    @Override
    public boolean getIntake() {
        return secondaryController.getTrigger();
    }

    @Override
    public boolean getRoller() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRoller'");
    }

    @Override
    public double getHoodDisplacement() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getHoodDisplacement'");
    }

}
