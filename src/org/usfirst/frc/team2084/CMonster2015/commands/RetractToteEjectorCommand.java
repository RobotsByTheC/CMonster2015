/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.commands;

import org.usfirst.frc.team2084.CMonster2015.Robot;
import org.usfirst.frc.team2084.CMonster2015.subsystems.ToteLifterSubsystem.EjectorState;
import org.usfirst.frc.team2084.CMonster2015.subsystems.ToteLifterSubsystem.GateState;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RetractToteEjectorCommand extends Command {

    public RetractToteEjectorCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.toteLifterSubsystem);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        Robot.toteLifterSubsystem.setEjectorState(EjectorState.RETRACTING);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return Robot.toteLifterSubsystem.isEjectorRetracted();
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.toteLifterSubsystem.setGateState(GateState.CLOSED);
        Robot.toteLifterSubsystem.setEjectorState(EjectorState.STOPPED);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
