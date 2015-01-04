/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2015.commands;

import org.usfirst.frc2084.CMonster2015.Robot;
import org.usfirst.frc2084.CMonster2015.drive.processors.InertiaGenerator;
import org.usfirst.frc2084.CMonster2015.drive.processors.RescalingDeadband;
import org.usfirst.frc2084.CMonster2015.drive.processors.Scaler;
import org.usfirst.frc2084.CMonster2015.drive.processors.TimeStepper;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The command that implements the control, filtering and scaling of our amazing
 * Mecanum drive system.
 *
 * @author Ben Wolsieffer
 */
public class FieldCentricMecanumDriveCommand extends Command {

	/**
	 * The joystick z-axis value below which the robot will not rotate. this is
	 * to prevent accidental small twists of the joystick from affecting its
	 * trajectory.
	 */
	public static final double ROTATION_DEADBAND = 0.2;
	/**
	 * The maximum speed that the robot is allowed to rotate at. The joystick
	 * value is scaled down to this value.
	 */
	public static final double MAX_ROTATION = 0.5;
	public static final double ROTATION_INERTIA_GAIN = 0.2;
	private final TimeStepper timeStepper = new TimeStepper();
	private final RescalingDeadband rotationDeadband = new RescalingDeadband(ROTATION_DEADBAND);
	private final Scaler rotationScaler = new Scaler(MAX_ROTATION);
	private final InertiaGenerator rotationInertiaGenerator = new InertiaGenerator(ROTATION_INERTIA_GAIN);

	public FieldCentricMecanumDriveCommand() {
		// This command drives, so it requires the drive subsytem.
		// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
		requires(Robot.driveSubsystem);
		// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	}

	/**
	 * Does nothing.
	 */
	@Override
	protected void initialize() {
		timeStepper.reset();
	}

	/**
	 * Updates the robot's speed based on the joystick values.
	 */
	@Override
	protected void execute() {
		// This is the joystick that we use as input for driving.
		Joystick driveJoystick = Robot.oi.getDriveJoystick();
		// Store the axis values.
		double x = driveJoystick.getX();
		double y = -driveJoystick.getY(); // Forward is negative on joysticks
		double rotation = driveJoystick.getZ();
		// This will hold the scaled rotation value. We scale down this value
		// because otherwise the robot is too hard ot control with the joystick
		// twist and we don't need our full possible rotation speed (its pretty
		// fast).
		double scaledRotation = rotationScaler.process(rotationDeadband.process(rotation));

		// Implement the rotation inertia generator. See InertiaGenerator for
		// more information.
		double actualRotation = rotationInertiaGenerator.process(scaledRotation, timeStepper.step());
		// Send debugging values.
		SmartDashboard.putNumber("Joystick X", x);
		SmartDashboard.putNumber("Joystick Y", y);
		SmartDashboard.putNumber("Joystick Rotation", rotation);
		SmartDashboard.putNumber("Scaled Rotation", scaledRotation);
		// Actually drive the robot using the joystick values for x and y and
		// the scaled z value. The inversions are necessary because of the way
		// the rest of the code is set. We shouldn't touch them until we have
		// time to go through and make sure we can fix all the unnecessary
		// inversions.
		Robot.driveSubsystem.getMecanumDriveAlgorithm().driveFieldCartesian(x, y, actualRotation);
	}

	/**
	 * This command never ends on its own but it could be interrupted, for
	 * example if we reverted back to our failsafe driving mode.
	 *
	 * @return false
	 */
	@Override
	protected boolean isFinished() {
		return false;
	}

	/**
	 * Stops the drive motors.
	 */
	@Override
	protected void end() {
		Robot.driveSubsystem.getMecanumDriveAlgorithm().stop();
	}

	/**
	 * Stops the drive motors.
	 */
	@Override
	protected void interrupted() {
		end();
	}
}
