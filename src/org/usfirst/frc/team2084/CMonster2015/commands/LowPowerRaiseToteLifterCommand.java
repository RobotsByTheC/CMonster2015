/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.commands;

import org.usfirst.frc.team2084.CMonster2015.Robot;
import org.usfirst.frc.team2084.CMonster2015.subsystems.ToteLifterSubsystem.LifterState;

import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 * Raises the tote lifter without venting the top of the cylinder. This reduces
 * the force of the raise while increasing the force of the retract. We thought
 * this would fix a problem, but it really does not.
 */
public class LowPowerRaiseToteLifterCommand extends TimedCommand {

    public LowPowerRaiseToteLifterCommand() {
        super(1);

        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.toteLifterSubsystem);

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    /**
     * Raises the tote lifter at low power.
     */
    @Override
    protected void initialize() {
        Robot.toteLifterSubsystem.setLifterState(LifterState.LOW_POWER_RAISE);
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
    }
}
