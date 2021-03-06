/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.commands;

import org.usfirst.frc.team2084.CMonster2015.Robot;

/**
 * Command that enables or disables the encoders for driving during teleop.
 */
public class SetEncodersEnabledCommand extends ParameterCommand {

    public static final String ENABLED_KEY = "Enabled";

    /**
     * Creates a {@link SetEncodersEnabledCommand} that enables the encoders.
     * This mostly exists to set the default for the SmartDashboard.
     */
    public SetEncodersEnabledCommand() {
        this(true);
    }

    /**
     * Creates a {@link SetGyroEnabledCommand} that enables or disables the
     * encoders.
     * 
     * @param enabled whether the encoders should be enabled
     */
    public SetEncodersEnabledCommand(boolean enabled) {
        setRunWhenDisabled(true);
        addBooleanParameter(ENABLED_KEY, enabled);

        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    /**
     * Enables or disables the encoders.
     */
    @Override
    protected void initialize() {
        Robot.driveSubsystem.setEncodersEnabled(getBooleanParameter(ENABLED_KEY));
    }

    @Override
    protected void execute() {
    }

    /**
     * @return true
     */
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
