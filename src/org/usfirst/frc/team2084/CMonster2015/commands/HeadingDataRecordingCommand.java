/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.commands;

import org.usfirst.frc.team2084.CMonster2015.Robot;
import org.usfirst.frc.team2084.CMonster2015.RobotMap;

/**
 *
 */
public class HeadingDataRecordingCommand extends DataRecordingCommand {

    public HeadingDataRecordingCommand() {
        super("heading");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);

        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.driveSubsystem);

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    @Override
    protected void execute() {
        double output = Robot.oi.getDriveJoystick().getX();

        RobotMap.driveSubsystemArcadeDriveAlgorithm.arcadeDrive(0, output);
        writeData(Double.toString(RobotMap.driveSubsystemGyro.getAngle()), Double.toString(output));
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
