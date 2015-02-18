/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.commands;

import org.usfirst.frc.team2084.CMonster2015.Robot;
import org.usfirst.frc.team2084.CMonster2015.RobotMap;
import org.usfirst.frc.team2084.CMonster2015.drive.Location;
import org.usfirst.frc.team2084.CMonster2015.drive.processors.LinearRamper;
import org.usfirst.frc.team2084.CMonster2015.drive.processors.LinearRamper.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Drives the robot to a specific location and heading. The speed is ramped to
 * minimize wheel slip which could throw off the pose tracking.
 * 
 * @author Ben Wolsieffer
 */
public class DriveToLocationCommand extends RotateToCommand {

    public static final String X_LOCATION_ERROR_KEY = "X Location Error";
    public static final String Y_LOCATION_ERROR_KEY = "Y Location Error";

    public static final String X_KEY = "X";
    public static final String Y_KEY = "Y";

    private Location location;

    private LinearRamper ramper = new LinearRamper(1, Type.UP);

    private double maxSpeed = 1.0;

    public DriveToLocationCommand() {
        this(new Location(0, 0), 0);
    }

    public DriveToLocationCommand(Location location, double heading) {
        this(location, heading, 1, 1);
    }

    public DriveToLocationCommand(Location location, double heading, double maxMovementSpeed, double maxRotationSpeed) {
        this(location, heading, maxMovementSpeed, maxRotationSpeed, 5, false);
    }

    /**
     * Creates a new {@link DriveToLocationCommand} to the specified location
     * and heading. It will timeout after the specified number of seconds.
     * 
     * @param location the location to drive to
     * @param heading the target heading
     * @param maxMovementSpeed the maximum speed to move
     * @param maxRotationSpeed the maximum speed to rotate
     * @param timeout the max time the robot can take to reach the target
     * @param debug whether to output the location and heading error
     */
    public DriveToLocationCommand(Location location, double heading, double maxMovementSpeed, double maxRotationSpeed, double timeout, boolean debug) {
        super(heading, maxRotationSpeed, timeout, debug);

        addNumberParameter(X_KEY, location.getX());
        addNumberParameter(Y_KEY, location.getY());

        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.driveSubsystem);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    /**
     * Resets the ramper and gets the parameter values.
     */
    @Override
    protected void initialize() {
        super.initialize();

        location = new Location(getNumberParameter(X_KEY), getNumberParameter(Y_KEY));
    }

    /**
     * Drives the robot to the location.
     */
    @Override
    protected void execute() {
        double maxOutput = Math.min(ramper.process(1.0), maxSpeed);
        RobotMap.driveSubsystemMecanumDriveAlgorithm.driveToLocation(location, heading, maxOutput, maxOutput);
        if (debug) {
            Location locationError = RobotMap.driveSubsystemMecanumDriveAlgorithm.getLocationError();
            SmartDashboard.putNumber(X_LOCATION_ERROR_KEY, locationError.getX());
            SmartDashboard.putNumber(Y_LOCATION_ERROR_KEY, locationError.getY());
            SmartDashboard.putNumber(HEADING_ERROR_KEY, RobotMap.driveSubsystemMecanumDriveAlgorithm.getHeadingError());
        }
    }

    /**
     * Returns true when the x and y locations, along with the heading, are on
     * target.
     */
    @Override
    protected boolean isFinished() {
        return RobotMap.driveSubsystemMecanumDriveAlgorithm.isLocationOnTarget() && super.isFinished();
    }
}
