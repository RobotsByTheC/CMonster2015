/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2014.subsystems;

import org.usfirst.frc2084.CMonster2014.RobotMap;
import org.usfirst.frc2084.CMonster2014.commands.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc2084.CMonster2014.drive.ArcadeDriveAlgorithm;
import org.usfirst.frc2084.CMonster2014.drive.FourWheelDriveController;
import org.usfirst.frc2084.CMonster2014.drive.MecanumDriveAlgorithm;
import org.usfirst.frc2084.CMonster2014.drive.WheelController;

/**
 *
 */
public class DriveSubsystem extends Subsystem {

    public static final double CRAB_SPEED = 0.6;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    SpeedController frontLeftJaguar = RobotMap.driveSubsystemFrontLeftJaguar;
    SpeedController frontRightJaguar = RobotMap.driveSubsystemFrontRightJaguar;
    SpeedController rearLeftJaguar = RobotMap.driveSubsystemRearLeftJaguar;
    SpeedController rearRightJaguar = RobotMap.driveSubsystemRearRightJaguar;
    Encoder rearRightEncoder = RobotMap.driveSubsystemRearRightEncoder;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    BetterGyro steeringGyro = RobotMap.driveSubsystemSteeringGyro;
    TempSensor steeringGyroTemp = RobotMap.driveSubsystemSteeringGyroTemp;
    ADXL345_I2C accelerometer = RobotMap.driveSubsystemAccelerometer;
    private final WheelController frontLeftWheel = new WheelController(frontLeftJaguar);
    private final WheelController frontRightWheel = new WheelController(frontRightJaguar);
    private final WheelController rearLeftWheel = new WheelController(rearLeftJaguar);
    private final WheelController rearRightWheel = new WheelController(rearRightJaguar);
    private final FourWheelDriveController driveController = new FourWheelDriveController(
            frontLeftWheel,
            frontRightWheel,
            rearLeftWheel,
            rearRightWheel
    );
    private final MecanumDriveAlgorithm mecanumDriveAlgorithm = new MecanumDriveAlgorithm(driveController, RobotMap.driveSubsystemSteeringGyro);
    private final ArcadeDriveAlgorithm arcadeDriveAlgorithm = new ArcadeDriveAlgorithm(driveController);

    public DriveSubsystem() {
        frontRightWheel.setInverted(true);
        rearRightWheel.setInverted(true);
    }

    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
        setDefaultCommand(new FieldCentricMecanumDriveCommand());
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
        steeringGyro.setPIDSourceParameter(PIDSource.PIDSourceParameter.kAngle);
    }

    public MecanumDriveAlgorithm getMecanumDriveAlgorithm() {
        return mecanumDriveAlgorithm;
    }

    public ArcadeDriveAlgorithm getArcadeDriveAlgorithm() {
        return arcadeDriveAlgorithm;
    }
}
