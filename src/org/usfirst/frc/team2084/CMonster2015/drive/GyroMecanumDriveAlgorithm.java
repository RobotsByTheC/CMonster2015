/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.drive;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;

import org.usfirst.frc.team2084.CMonster2015.Gyro;
import org.usfirst.frc.team2084.neuralnetwork.Data;
import org.usfirst.frc.team2084.neuralnetwork.Data.FormatException;
import org.usfirst.frc.team2084.neuralnetwork.Network;
import org.usfirst.frc.team2084.neuralnetwork.Neuron;
import org.usfirst.frc.team2084.neuralnetwork.TransferFunction;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * A {@link DriveAlgorithm} that uses a {@link FourWheelDriveController} (which
 * controls four mecanum wheels) to drive using a gyro to maintain orientation
 * and drive relative to the field. Mecanum wheels have rollers set at a 45
 * degree angle from the wheel's direction of rotation. This allows them the
 * robot to move in any direction.
 *
 * @see FourWheelDriveController
 * @see DriveAlgorithm
 *
 * @author Ben Wolsieffer
 */
public class GyroMecanumDriveAlgorithm<S extends WheelController<?>> extends MecanumDriveAlgorithm<S> {

    /**
     * The rotation speed below which the heading controller is enabled. When
     * the rotation speed increases above this value the controller is disabled
     * to allow the robot to turn.
     */
    public static final double ROTATION_DEADBAND = 0.05;

    public static final String HEADING_NETWORK_PATH = "/home/lvuser/GyroMecanumDriveAlgorithm.headingNetwork.txt";
    public static final int[] HEADING_NETWORK_TOPOLOGY = { 2, 3, 1, 1 };
    public static final double HEADING_NETWORK_ETA = 0.15;
    public static final double HEADING_NETWORK_MOMENTUM = 0.05;
    public static final TransferFunction HEADING_NETWORK_TRANSFER_FUNCTION = new TransferFunction.HyperbolicTangent();

    /**
     * The {@link Gyro} that the {@link GyroMecanumDriveAlgorithm} uses for
     * field-oriented driving and keeping the correct orientation.
     */
    protected final Gyro gyro;

    /**
     * Stores the heading offset used for robot centric driving.
     */
    private double headingOffset = 0.0;

    /**
     * Flags that stores whether the gyro should be enabled.
     */
    private boolean gyroEnabled = true;

    private double headingInverted = 1.0;

    private Data headingNetworkData;

    /**
     * Neural network that maintains the orientation of the robot using the
     * gyro.
     */
    private final Network headingNetwork;
    private final Neuron headingNetworkOutputNeuron;
    private final Neuron headingNetworkRotationSpeedNeuron;
    private boolean headingNetworkEnabled = false;
    private boolean headingNetworkTrainingEnabled = false;
    private double headingNetworkSetpoint;
    private final double headingTolerance;

    /**
     * Creates a new {@link GyroMecanumDriveAlgorithm} that controls the
     * specified {@link FourWheelDriveController}.
     *
     * @param controller the {@link FourWheelDriveController} to control
     * @param gyro the {@link SynchronizedRadianGyro} to use for orientation
     *            correction and field-oriented driving
     * @param headingTolerance the tolerance (in radians) to consider as on
     *            target
     */
    public GyroMecanumDriveAlgorithm(FourWheelDriveController<S> controller, Gyro gyro, double headingTolerance) {
        super(controller);
        this.gyro = gyro;
        this.headingTolerance = headingTolerance;

        File headingNetworkFile = new File(HEADING_NETWORK_PATH);

        if (headingNetworkFile.canRead()) {
            try {
                headingNetworkData = new Data(headingNetworkFile);
            } catch (FormatException e) {
                System.err.println("Heading network file contains errors: " + e);
            } catch (FileNotFoundException e) {
                System.err.println("Heading network file cannot be found, this shouldn't happen: " + e);
            }
        }

        if (headingNetworkData != null) {
            headingNetwork = headingNetworkData.getNetwork();
        } else {
            headingNetwork = new Network(HEADING_NETWORK_TOPOLOGY, HEADING_NETWORK_ETA, HEADING_NETWORK_MOMENTUM, HEADING_NETWORK_TRANSFER_FUNCTION);
            headingNetworkData = new Data(headingNetwork);
        }

        headingNetworkOutputNeuron = headingNetwork.getLayer(headingNetwork.getTotalLayers() - 1)[0];
        headingNetworkRotationSpeedNeuron = headingNetwork.getLayer(headingNetwork.getTotalLayers() - 2)[0];
    }

    /**
     * {@inheritDoc} This also uses the gyro to keep the robot going in a
     * straight line.
     */
    @Override
    public void driveCartesian(double x, double y, double rotation) {
        driveFieldCartesianImplCorrection(x, y, rotation, getHeading() - headingOffset);
    }

    /**
     * Moves the robot forward and sideways at the specified speeds.
     *
     * @param x the forward speed (negative = backward, positive = forward)
     * @param y the sideways (crab) speed (negative = left, positive = right)
     *
     */
    public void driveFieldCartesian(double x, double y) {
        driveFieldCartesian(x, y, 0);
    }

    /**
     * Moves the robot forward and sideways while rotating at the specified
     * speeds. This moves the robot relative to the field.
     *
     * @param x the forward speed (negative = backward, positive = forward)
     * @param y the sideways (crab) speed (negative = left, positive = right)
     * @param rotation the speed to rotate at while moving (negative =
     *            clockwise, positive = counterclockwise)
     */
    public void driveFieldCartesian(double x, double y, double rotation) {
        driveFieldCartesianImplCorrection(x, y, rotation, getHeading());
    }

    /**
     * Drive based on the specified joystick using the x and y and twist axes.
     *
     * @param stick the joystick to use
     */
    public void driveFieldCartesian(GenericHID stick) {
        driveFieldCartesian(stick.getX(), stick.getY(), stick.getTwist());
    }

    /**
     * Drives the robot at the specified x and y speeds relative to the field
     * while maintaining the specified heading.
     * 
     * @param x the x speed
     * @param y the y speed
     * @param heading the heading to maintain
     */
    public void driveFieldHeadingCartesian(double x, double y, double heading) {
        driveFieldHeadingCartesian(x, y, heading, 1.0);
    }

    /**
     * Drives the robot at the specified x and y speeds relative to the field
     * while maintaining the specified heading. It also limits the maximum
     * rotation speed.
     * 
     * @param x the x speed
     * @param y the y speed
     * @param heading the heading to maintain
     * @param maxRotationSpeed the maximum speed rotation speed
     */
    public void driveFieldHeadingCartesian(double x, double y, double heading, double maxRotationSpeed) {
        driveFieldCartesianImplNoCorrection(x, y, getHeadingCorrection(heading, maxRotationSpeed), getHeading());
    }

    /**
     * Private implementation of field-oriented cartesian mecanum driving that
     * accounts for heading correction.
     *
     * @param x the forward speed (negative = backward, positive = forward)
     * @param y the sideways (crab) speed (negative = left, positive = right)
     * @param rotation The speed to rotate at while moving (negative =
     *            clockwise, positive = counterclockwise)
     * @param gyroAngle the current angle reading from the gyro
     */
    private void driveFieldCartesianImplCorrection(double x, double y, double rotation, double gyroAngle) {
        rotation = getRotationCorrection(rotation);
        driveFieldCartesianImplNoCorrection(x, y, rotation, gyroAngle);
    }

    /**
     * Private implementation of field-oriented cartesian mecanum driving that
     * does not account for heading correction.
     *
     * @param x the forward speed (negative = backward, positive = forward)
     * @param y the sideways (crab) speed (negative = left, positive = right)
     * @param rotation The speed to rotate at while moving (negative =
     *            clockwise, positive = counterclockwise)
     * @param gyroAngle the current angle reading from the gyro
     */
    private void driveFieldCartesianImplNoCorrection(double x, double y, double rotation, double gyroAngle) {
        // Compensate for gyro angle.
        double rotated[] = DriveUtils.rotateVector(x, y, gyroAngle);
        x = rotated[0];
        y = rotated[1];

        super.driveCartesian(x, y, rotation);
    }

    /**
     * Drives the robot at the specified speed and in the specified direction,
     * while maintaining the specified heading.
     * 
     * @param magnitude the movement speed
     * @param direction the movement direction
     * @param heading the heading to maintain
     * @return true when the heading is on target
     */
    public boolean driveFieldHeadingPolar(double magnitude, double direction, double heading) {
        return driveFieldHeadingPolar(magnitude, direction, heading, 1.0);
    }

    /**
     * Drives the robot at the specified speed and in the specified direction,
     * while maintaining the specified heading. It also limits the maximum
     * rotation speed.
     * 
     * @param magnitude the movement speed
     * @param direction the movement direction
     * @param heading the heading to maintain
     * @param maxRotationSpeed the maximum speed rotation speed
     * @return true when the heading is on target
     */
    public boolean driveFieldHeadingPolar(double magnitude, double direction, double heading, double maxRotationSpeed) {
        driveFieldPolarImplNoCorrection(magnitude, direction, getHeadingCorrection(heading, maxRotationSpeed), getHeading());
        return isHeadingOnTarget();
    }

    /**
     * Drives the robot at the specified speed in the direction specified as an
     * angle in radians. This theoretically takes into account the gyro
     * correction, but it has not been tested because we do not use it.
     *
     * @param magnitude the speed that the robot should drive in a given
     *            direction.
     * @param direction the direction the robot should drive in radians,
     *            independent of rotation
     */
    public void driveFieldPolar(double magnitude, double direction) {
        driveFieldPolar(magnitude, direction, 0);
    }

    /**
     * Drives the robot at the specified speed in the direction specified as an
     * angle in radians. This theoretically takes into account the gyro
     * correction, but it has not been tested because we do not use it.
     *
     * @param magnitude the speed that the robot should drive in a given
     *            direction.
     * @param direction the direction the robot should drive in radians,
     *            independent of rotation
     * @param rotation the rate of rotation for the robot that is completely
     *            independent of the magnitude or direction. [-1.0..1.0]
     */
    public void driveFieldPolar(double magnitude, double direction, double rotation) {
        driveFieldPolarImplCorrection(magnitude, direction, rotation, getHeading());
    }

    /**
     * Private implementation of field-oriented polar mecanum driving that
     * accounts for heading correction.
     *
     * @param magnitude the speed that the robot should drive in a given
     *            direction.
     * @param direction the direction the robot should drive in radians,
     *            independent of rotation
     * @param rotation the rate of rotation for the robot that is completely
     *            independent of the magnitude or direction. [-1.0..1.0]
     * @param gyroAngle the current angle reading from the gyro
     */
    private void driveFieldPolarImplCorrection(double magnitude, double direction, double rotation, double gyroAngle) {
        rotation = getRotationCorrection(rotation);
        driveFieldPolarImplNoCorrection(magnitude, direction, rotation, gyroAngle);
    }

    /**
     * Private implementation of field-oriented polar mecanum driving that does
     * not account for heading correction.
     *
     * @param magnitude the speed that the robot should drive in a given
     *            direction.
     * @param direction the direction the robot should drive in radians,
     *            independent of rotation
     * @param rotation the rate of rotation for the robot that is completely
     *            independent of the magnitude or direction. [-1.0..1.0]
     * @param gyroAngle the current angle reading from the gyro
     */
    private void driveFieldPolarImplNoCorrection(double magnitude, double direction, double rotation, double gyroAngle) {
        direction += gyroAngle;
        drivePolar(magnitude, direction, rotation);
    }

    /**
     * Moves the robot sideways at the specified speed.
     *
     * @param speed The speed and direction to crab (negative = left, positive =
     *            right)
     */
    public void crab(double speed) {
        driveCartesian(speed, 0);
    }

    /**
     * Gets the proper rotation speed to face the robot to the specified
     * heading.
     * 
     * @param heading
     * @param maxRotationSpeed
     * @return
     */
    private double getHeadingCorrection(double heading, double maxRotationSpeed) {
        headingNetworkEnabled = true;
        headingNetworkSetpoint = heading;
        if (headingNetworkTrainingEnabled) {
            headingNetworkOutputNeuron.setOutputValue(getHeadingError());
            headingNetwork.backPropagation(0);
            System.out.println("training!!!");
            System.out.println("error: " + headingNetwork.getRecentAverageError());
        }
        headingNetwork.feedForward(getHeadingError(), maxRotationSpeed);

        double rotationSpeed = -headingNetworkRotationSpeedNeuron.getOutputValue();

        double rotationSpeedSign = rotationSpeed > 0 ? 1 : -1;
        return Math.abs(rotationSpeed) > maxRotationSpeed ? maxRotationSpeed * rotationSpeedSign : rotationSpeed;
    }

    /**
     * Gets the corrected rotation speed based on the gyro heading and the
     * expected rate of rotation. If the rotation rate is above a threshold, the
     * gyro correction is turned off.
     *
     * @param rotationSpeed the speed at which the robot is trying to rotate
     * @return the corrected rotation speed
     */
    private double getRotationCorrection(double rotationSpeed) {
        if (gyroEnabled) {
            // If the controller is already enabled, check to see if it should
            // be disabled or kept running. Otherwise check to see if it needs
            // to be enabled.
            if (headingNetworkEnabled) {
                // If the rotation rate is greater than the deadband disable the
                // heading controller. Otherwise, return the latest value from
                // the controller.
                if (Math.abs(rotationSpeed) >= ROTATION_DEADBAND) {
                    headingNetworkEnabled = false;
                } else {
                    if (headingNetworkTrainingEnabled) {
                        headingNetworkOutputNeuron.setOutputValue(getHeadingError());
                        headingNetwork.backPropagation(headingNetworkSetpoint);
                    }
                    headingNetwork.feedForward(headingNetworkSetpoint, 1.0);
                    return headingNetworkRotationSpeedNeuron.getOutputValue();
                }
            } else {
                // If the rotation rate is less than the deadband, turn on the
                // PID controller and set its setpoint to the current angle.
                if (Math.abs(rotationSpeed) < ROTATION_DEADBAND) {
                    headingOffset = getHeading();
                    headingNetworkSetpoint = headingOffset;
                    headingNetworkEnabled = true;
                }
            }
        }
        // Unless told otherwise, return the rate that was passed in.
        return rotationSpeed;
    }

    /**
     * Resets the robot's gyro value to zero. This is usually called on command,
     * or after the robot has been disabled to get rid of drift.
     */
    public void resetGyro() {
        // Reset the gyro value to zero
        gyro.reset();
        // Reset the integral component to zero (which also disables the
        // controller). This is very important because the integral value will
        headingNetworkSetpoint = 0;
    }

    public void resetSetpoint() {
        headingNetworkSetpoint = getHeading();
    }

    /**
     * Gets the heading of the robot in radians according to the gyro. This also
     * inverts the value if necessary. This *must* be used to retrieve the gyro
     * heading rather than calling {@link Gyro#getAngle()} to prevent race
     * conditions.
     * 
     * @return the heading
     */
    public double getHeading() {
        synchronized (this) {
            return DriveUtils.normalizeHeading(gyro.getAngle() * headingInverted);
        }
    }

    /**
     * Sets the heading of the robot. This should be called rather than
     * {@link Gyro#setAngle(double)} to prevent the robot from trying to rotate
     * to this new heading, which is generally not the desired behavior.
     * 
     * @param heading
     */
    public void setHeading(double heading) {
        synchronized (this) {
            gyro.setAngle(DriveUtils.normalizeHeading(heading * headingInverted));
        }
        // This must not be synchronized to avoid deadlock
        resetSetpoint();
    }

    public double getRotationRate() {
        synchronized (this) {
            return gyro.getRate() * headingInverted;
        }
    }

    /**
     * Sets whether the neural network should train itself.
     * 
     * @param enabled whether training should be enabled
     */
    public void setTrainingEnabled(boolean enabled) {
        headingNetworkTrainingEnabled = enabled;
    }

    /**
     * Gets whether neural network training is enabled.
     * 
     * @return true if the network is enabled
     */
    public boolean isTrainingEnabled() {
        return headingNetworkTrainingEnabled;
    }

    /**
     * Saves the state of the neural network. This should be called whenever
     * training finishes.
     */
    public void saveTraining() {
        try {
            headingNetworkData.save(new File(HEADING_NETWORK_PATH));
        } catch (IOException e) {
            System.err.println("Unable to save heading network training data.");
        }
    }

    public void setHeadingInverted(boolean inverted) {
        headingInverted = inverted ? -1.0 : 1.0;
    }

    public boolean isHeadingInverted() {
        return headingInverted == -1.0;
    }

    /**
     * Gets the angular speed of the robot in radians/second according to the
     * gyro.
     * 
     * @return the angular speed
     */
    public double getAngularSpeed() {
        synchronized (this) {
            return gyro.getRate() * headingInverted;
        }
    }

    /**
     * Gets whether the robot is facing the direction it should be. This always
     * returns true if the robot is being commanded to spinat a certain rate.
     * 
     * @return true if the robot is on target
     */
    public boolean isHeadingOnTarget() {
        if (headingNetworkEnabled) {
            return Math.abs(getHeadingError()) < headingTolerance;
        } else {
            return true;
        }
    }

    /**
     * Sets whether the algorithm should use the gyro. It would be disabled if
     * it fails during a match. Methods that require a gyro for basic
     * functionality are not affected by this.
     * 
     * @param enabled whether the gyro should be enabled
     */
    public void setGyroEnabled(boolean enabled) {
        gyroEnabled = enabled;
    }

    /**
     * Gets whether the gyro is enabled.
     * 
     * @see #setGyroEnabled(boolean)
     * @return whether the gyro is enabled
     */
    public boolean isGyroEnabled() {
        return gyroEnabled;
    }

    public double getHeadingError() {
        return headingNetworkSetpoint - getHeading();
    }

    /**
     * Gets the most recent back-propagation error from the neural network.
     * 
     * @return the back-propagation error
     */
    public double getBackPropagationError() {
        return headingNetwork.getRecentAverageError();
    }
}
