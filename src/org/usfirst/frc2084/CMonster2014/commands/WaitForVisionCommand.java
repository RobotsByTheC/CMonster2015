package org.usfirst.frc2084.CMonster2014.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc2084.CMonster2014.TargetTrackingCommunication;

/**
 *
 */
public class WaitForVisionCommand extends Command {

    /**
     * The maximum amount of time to wait for information from the vision
     * processing algorithm on the driver station laptop.
     */
    private static final double MAX_TIME = 1.0;

    public WaitForVisionCommand() {
        super(MAX_TIME);
        setTimeout(MAX_TIME);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        TargetTrackingCommunication.setState(TargetTrackingCommunication.State.UNKNOWN);
        // Last ditch effort to bring the camera up on the DS laptop, probably
        // is too late.
        TargetTrackingCommunication.setCameraEnabled(true);
        // Tell the DS laptop to starting detecting the hot target
        TargetTrackingCommunication.setAutonomousVisionRunning(true);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return !TargetTrackingCommunication.getState().equals(TargetTrackingCommunication.State.UNKNOWN) || timeSinceInitialized() > MAX_TIME;
    }

    // Called once after isFinished returns true
    protected void end() {
        TargetTrackingCommunication.setAutonomousVisionRunning(false);
        TargetTrackingCommunication.setCameraEnabled(false);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
