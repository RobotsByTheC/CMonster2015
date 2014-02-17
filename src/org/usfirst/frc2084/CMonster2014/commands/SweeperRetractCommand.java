 // RobotBuilder Version: 1.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.
package org.usfirst.frc2084.CMonster2014.commands;
import org.usfirst.frc2084.CMonster2014.Robot;
/**
 *
 */
public class SweeperRetractCommand extends TimedCommand {
    public SweeperRetractCommand() {
        super(0.75);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.sweeperSubsystem);
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }
    protected void run() {
        Robot.sweeperSubsystem.retract();
    }
    protected void end() {
    }
    protected void interrupted() {
    }
}
