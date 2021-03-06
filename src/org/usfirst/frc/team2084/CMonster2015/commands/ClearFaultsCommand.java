/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.commands;

import org.usfirst.frc.team2084.CMonster2015.Robot;
import org.usfirst.frc.team2084.CMonster2015.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Clears sticky faults of the PDP and the PCM.
 */
public class ClearFaultsCommand extends Command {

    public ClearFaultsCommand() {
        setRunWhenDisabled(true);
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    /**
     * Clears the sticky faults.
     */
    @Override
    protected void initialize() {
        Robot.pdp.clearStickyFaults();
        RobotMap.toteLifterSubsystemGateSolenoid.clearAllPCMStickyFaults();
    }

    @Override
    protected void execute() {
    }

    @Override
    protected boolean isFinished() {
        return true;
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
    }
}
