package frc.robot.controllers;

public interface DriveController {
    /**
     * Gets the x speed from the controller, in the range [-1, 1].
     * This is the forward/backward speed of the robot, where positive is forward.
     * @return the x speed from the controller
     */
    public double getDriveX();

    /**
     * Gets the y speed from the controller, in the range [-1, 1].
     * This is the left/right speed of the robot, where positive is to the right.
     * @return the y speed from the controller
     */
    public double getDriveY();

    /**
     * Gets the turn speed from the controller, in the range [-1, 1].
     * This is the rotational speed of the robot, where positive is clockwise.
     * @return the turn speed from the controller
     */
    public double getDriveTurn();

    /**
     * Gets the boost multiplier from the controller, in the range [0, 1].
     * This is used to increase the speed of the robot when a boost button is held.
     * @return the boost multiplier from the controller
     */
    public double getDriveBoost();

    /**
     * Gets whether the controller is in field-centric mode.
     * @return whether the controller is in field-centric mode
     */
    public boolean getDriveFieldCentricMode();

    /**
     * Gets whether the controller is in field-centric facing origin mode.
     * @return whether the controller is in field-centric facing origin mode
     */
    public boolean getDriveFieldCentricFacingOriginMode();

    /**
     * Gets whether the controller is in field-centric snapping mode.
     * @return whether the controller is in field-centric snapping mode
     */
    public boolean getDriveFieldCentricSnappingMode();

    /**
     * Gets whether the reset pose button is pressed.
     * @return whether the reset pose button is pressed
     */
    public boolean getResetPoseButton();

    /**
     * Gets whether the align to origin pose button is pressed.
     * @return whether the align to origin pose button is pressed
     */
    public boolean getAlignToOriginPoseButton();

    public boolean getShoot();

    public boolean getIntake();
}
