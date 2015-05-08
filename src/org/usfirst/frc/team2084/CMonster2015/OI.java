/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015;

import org.usfirst.frc.team2084.CMonster2015.commands.ClearFaultsCommand;
import org.usfirst.frc.team2084.CMonster2015.commands.CloseToteGateCommand;
import org.usfirst.frc.team2084.CMonster2015.commands.DriveHeadingCommand;
import org.usfirst.frc.team2084.CMonster2015.commands.DriveToLocationCommand;
import org.usfirst.frc.team2084.CMonster2015.commands.ExtendToteEjectorCommand;
import org.usfirst.frc.team2084.CMonster2015.commands.FallbackDriveCommandGroup;
import org.usfirst.frc.team2084.CMonster2015.commands.LowPowerRaiseToteLifterCommand;
import org.usfirst.frc.team2084.CMonster2015.commands.LowerContainerHookCommand;
import org.usfirst.frc.team2084.CMonster2015.commands.LowerToteLifterCommand;
import org.usfirst.frc.team2084.CMonster2015.commands.NormalDriveCommandGroup;
import org.usfirst.frc.team2084.CMonster2015.commands.OpenToteGateCommand;
import org.usfirst.frc.team2084.CMonster2015.commands.RaiseContainerHookCommand;
import org.usfirst.frc.team2084.CMonster2015.commands.RaiseToteLifterCommand;
import org.usfirst.frc.team2084.CMonster2015.commands.ResetGyroCommand;
import org.usfirst.frc.team2084.CMonster2015.commands.ResetHeadingSetpointCommand;
import org.usfirst.frc.team2084.CMonster2015.commands.RetractToteEjectorCommand;
import org.usfirst.frc.team2084.CMonster2015.commands.RotateToCommand;
import org.usfirst.frc.team2084.CMonster2015.commands.SetEncodersEnabledCommand;
import org.usfirst.frc.team2084.CMonster2015.commands.SetGyroEnabledCommand;
import org.usfirst.frc.team2084.CMonster2015.commands.SetHeadingCommand;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public JoystickButton normalDriveButton;
    public JoystickButton fallbackDriveButton;
    public JoystickButton resetGyroButton;
    public JoystickButton extendToteEjectorButton;
    public JoystickButton retractToteEjectorButton;
    public Joystick driveJoystick;
    public JoystickButton raiseToteLifterButton;
    public JoystickButton lowerToteLifterButton;
    public JoystickButton raiseContainerHookButton;
    public JoystickButton lowerContainerHookButton;
    public JoystickButton closeToteGateButton;
    public JoystickButton openToteGateButton;
    public Joystick secondaryJoystick;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public OI() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        secondaryJoystick = new Joystick(1);

        openToteGateButton = new JoystickButton(secondaryJoystick, 2);
        openToteGateButton.whenPressed(new OpenToteGateCommand());
        closeToteGateButton = new JoystickButton(secondaryJoystick, 4);
        closeToteGateButton.whenPressed(new CloseToteGateCommand());
        lowerContainerHookButton = new JoystickButton(secondaryJoystick, 8);
        lowerContainerHookButton.whileHeld(new LowerContainerHookCommand());
        raiseContainerHookButton = new JoystickButton(secondaryJoystick, 6);
        raiseContainerHookButton.whileHeld(new RaiseContainerHookCommand());
        lowerToteLifterButton = new JoystickButton(secondaryJoystick, 7);
        lowerToteLifterButton.whenPressed(new LowerToteLifterCommand());
        raiseToteLifterButton = new JoystickButton(secondaryJoystick, 5);
        raiseToteLifterButton.whenPressed(new RaiseToteLifterCommand());
        driveJoystick = new Joystick(0);

        retractToteEjectorButton = new JoystickButton(driveJoystick, 5);
        retractToteEjectorButton.whileHeld(new RetractToteEjectorCommand());
        extendToteEjectorButton = new JoystickButton(driveJoystick, 6);
        extendToteEjectorButton.whileHeld(new ExtendToteEjectorCommand());
        resetGyroButton = new JoystickButton(driveJoystick, 12);
        resetGyroButton.whenPressed(new ResetGyroCommand());
        fallbackDriveButton = new JoystickButton(driveJoystick, 10);
        fallbackDriveButton.whenPressed(new FallbackDriveCommandGroup());
        normalDriveButton = new JoystickButton(driveJoystick, 9);
        normalDriveButton.whenPressed(new NormalDriveCommandGroup());

        // SmartDashboard Buttons
        SmartDashboard.putData("Rotate To Command", new RotateToCommand());

        SmartDashboard.putData("Drive Heading Command", new DriveHeadingCommand());

        SmartDashboard.putData("Drive To Location Command", new DriveToLocationCommand());

        SmartDashboard.putData("Low Power Raise Tote Lifter Command", new LowPowerRaiseToteLifterCommand());

        SmartDashboard.putData("Clear Faults Command", new ClearFaultsCommand());

        SmartDashboard.putData("Set Gyro Enabled Command", new SetGyroEnabledCommand());

        SmartDashboard.putData("Set Encoders Enabled Command", new SetEncodersEnabledCommand());

        SmartDashboard.putData("Reset Gyro Command", new ResetGyroCommand());

        SmartDashboard.putData("Set Heading Command", new SetHeadingCommand());

        SmartDashboard.putData("Reset Heading Setpoint Command", new ResetHeadingSetpointCommand());

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
    public Joystick getDriveJoystick() {
        return driveJoystick;
    }

    public Joystick getSecondaryJoystick() {
        return secondaryJoystick;
    }

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
}
