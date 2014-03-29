// RobotBuilder Version: 1.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.
package org.usfirst.frc2084.CMonster2014;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc2084.CMonster2014.commands.*;
import org.usfirst.frc2084.CMonster2014.subsystems.*;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    private Command autonomousCommand;
    private Command funCommand;
    private final SendableChooser autonomousChooser = new SendableChooser();
    public static OI oi;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static DriveSubsystem driveSubsystem;
    public static CompressorSubsystem compressorSubsystem;
    public static SweeperSubsystem sweeperSubsystem;
    public static CatcherSubsytem catcherSubsytem;
    public static LedSubsystem ledSubsystem;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        RobotMap.init();
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        driveSubsystem = new DriveSubsystem();
        compressorSubsystem = new CompressorSubsystem();
        sweeperSubsystem = new SweeperSubsystem();
        catcherSubsytem = new CatcherSubsytem();
        ledSubsystem = new LedSubsystem();
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // This MUST be here. If the OI creates Commands (which it very likely
        // will), constructing it during the construction of CommandBase (from
        // which commands extend), subsystems are not guaranteed to be
        // yet. Thus, their requires() statements may grab null pointers. Bad
        // news. Don't move it.
        oi = new OI();
        // instantiate the command used for the autonomous period
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS
        funCommand = new FunCommand();
        funCommand.start();
        autonomousChooser.addDefault("Front Left Goal", new FrontAutonomousCommandGroup(true));
        autonomousChooser.addObject("Front Right Goal", new FrontAutonomousCommandGroup(false));
        SmartDashboard.putData("Autonomous Mode", autonomousChooser);
        // Make sure the DS laptop is not reporting vision to the robot
        TargetTrackingCommunication.setAutonomousVisionRunning(false);
        // Enable the camera on the DS laptop when the robot starts
        TargetTrackingCommunication.setCameraEnabled(true);
        Robot.ledSubsystem.sendCode(LedSubsystem.DISABLE_CODE);
    }
    public void autonomousInit() {
//        // Last ditch effort to bring the camera up on the DS laptop, probably
//        // is too late.
//        TargetTrackingCommunication.setCameraEnabled(true);
//        // Tell the DS laptop to starting detecting the hot target
//        TargetTrackingCommunication.setAutonomousVisionRunning(true);
//        Removed this and put it in the autonomous command
//        Robot.driveSubsystem.getRobotDrive().resetGyro();
        Object selection = autonomousChooser.getSelected();
        if (selection != null && selection instanceof Command) {
            autonomousCommand = (Command) selection;
            autonomousCommand.start();
        } else {
            System.out.println("No autonomous mode selected.");
        }
        Alliance alliance = DriverStation.getInstance().getAlliance();
        if (alliance.value == Alliance.kBlue_val) {
            Robot.ledSubsystem.sendCode(LedSubsystem.SOLID_BLUE_CODE);
        } else if (alliance.value == Alliance.kRed_val) {
            Robot.ledSubsystem.sendCode(LedSubsystem.SOLID_RED_CODE);
        } else {
            Robot.ledSubsystem.sendCode(LedSubsystem.DISABLE_CODE);
        }
    }
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }
    public void teleopInit() {
        TargetTrackingCommunication.setAutonomousVisionRunning(false);
        TargetTrackingCommunication.setCameraEnabled(false);
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to 
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        Robot.driveSubsystem.getRobotDrive().resetGyro();
    }
    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }
    /**
     * This function called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
    }
    public void disabledInit() {
        Robot.ledSubsystem.sendCode(LedSubsystem.DISABLE_CODE);
    }
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }
}
