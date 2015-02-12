/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015;

import org.usfirst.frc.team2084.CMonster2015.drive.ArcadeDriveAlgorithm;
import org.usfirst.frc.team2084.CMonster2015.drive.DIOEncoderWheelController;
import org.usfirst.frc.team2084.CMonster2015.drive.EncoderGyroMecanumDriveAlgorithm;
import org.usfirst.frc.team2084.CMonster2015.drive.EncoderWheelController;
import org.usfirst.frc.team2084.CMonster2015.drive.FourWheelDriveController;
import org.usfirst.frc.team2084.CMonster2015.drive.PIDConstants;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.PIDSource.PIDSourceParameter;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

    /**
     * The maximum speed the wheels can travel in meters/second.
     */
    public static final double DRIVE_SUBSYSTEM_MAX_WHEEL_SPEED = 1.0;
    public static final PIDConstants DRIVE_SUBSYSTEM_WHEEL_SPEED_PID_CONSTANTS = new PIDConstants(0, 0, 0, 1);
    public static final PIDConstants DRIVE_SUBSYSTEM_MECANUM_HEADING_PID_CONSTANTS = new PIDConstants(0.573, 0, 0);
    public static final PIDConstants DRIVE_SUBSYSTEM_X_LOCATION_PID_CONSTANTS = new PIDConstants(0, 0, 0);
    public static final PIDConstants DRIVE_SUBSYSTEM_Y_LOCATION_PID_CONSTANTS = new PIDConstants(0, 0, 0);
    public static final double DRIVE_SUBSYSTEM_LOCATION_TOLERANCE = 0.1;
    public static final double DRIVE_SUBSYSTEM_HEADING_TOLERANCE = 0.1;

    /**
     * The length of the drive base in meters.
     */
    public static final double DRIVE_SUBSYSTEM_DRIVE_BASE_LENGTH = 0.5969;

    /**
     * The width of the drive base in meters.
     */
    public static final double DRIVE_SUBSYSTEM_DRIVE_BASE_WIDTH = 0.5969;

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static SpeedController driveSubsystemFrontLeftJaguar;
    public static Encoder driveSubsystemFrontLeftEncoder;
    public static SpeedController driveSubsystemFrontRightJaguar;
    public static Encoder driveSubsystemFrontRightEncoder;
    public static SpeedController driveSubsystemRearLeftJaguar;
    public static Encoder driveSubsystemRearLeftEncoder;
    public static SpeedController driveSubsystemRearRightJaguar;
    public static Encoder driveSubsystemRearRightEncoder;
    public static Solenoid toteLifterSubsystemRaiseSolenoid;
    public static Solenoid toteLifterSubsystemLowerSolenoid;
    public static Solenoid toteLifterSubsystemGateSolenoid;
    public static DigitalInput toteLifterSubsystemEjectorExtendedLimitSwitch;
    public static DigitalInput toteLifterSubsystemEjectorRetractedLimitSwitch;
    public static SpeedController toteLifterSubsystemEjectorTalon;
    public static DigitalInput containerHookSubsystemUpperLimitSwitch;
    public static DigitalInput containerHookSubsystemLowerLimitSwitch;
    public static SpeedController containerHookSubsystemTalon;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static AnalogGyro driveSubsystemGyro;
    public static Accelerometer driveSubsystemAccelerometer;

    public static EncoderWheelController<SpeedController> driveSubsystemFrontLeftWheel;
    public static EncoderWheelController<SpeedController> driveSubsystemFrontRightWheel;
    public static EncoderWheelController<SpeedController> driveSubsystemRearLeftWheel;
    public static EncoderWheelController<SpeedController> driveSubsystemRearRightWheel;
    public static FourWheelDriveController<EncoderWheelController<SpeedController>> driveSubsystemDriveController;
    public static EncoderGyroMecanumDriveAlgorithm<EncoderWheelController<SpeedController>> driveSubsystemMecanumDriveAlgorithm;
    public static ArcadeDriveAlgorithm driveSubsystemArcadeDriveAlgorithm;

    // public static LEDController ledController;

    public static void init() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        driveSubsystemFrontLeftJaguar = new Jaguar(0);
        LiveWindow.addActuator("Drive Subsystem", "Front Left Jaguar", (Jaguar) driveSubsystemFrontLeftJaguar);

        driveSubsystemFrontLeftEncoder = new Encoder(0, 1, false, EncodingType.k4X);
        LiveWindow.addSensor("Drive Subsystem", "Front Left Encoder", driveSubsystemFrontLeftEncoder);
        driveSubsystemFrontLeftEncoder.setDistancePerPulse(0.00177325);
        driveSubsystemFrontLeftEncoder.setPIDSourceParameter(PIDSourceParameter.kRate);
        driveSubsystemFrontRightJaguar = new Jaguar(1);
        LiveWindow.addActuator("Drive Subsystem", "Front Right Jaguar", (Jaguar) driveSubsystemFrontRightJaguar);

        driveSubsystemFrontRightEncoder = new Encoder(2, 3, false, EncodingType.k4X);
        LiveWindow.addSensor("Drive Subsystem", "Front Right Encoder", driveSubsystemFrontRightEncoder);
        driveSubsystemFrontRightEncoder.setDistancePerPulse(0.00177325);
        driveSubsystemFrontRightEncoder.setPIDSourceParameter(PIDSourceParameter.kRate);
        driveSubsystemRearLeftJaguar = new Jaguar(2);
        LiveWindow.addActuator("Drive Subsystem", "Rear Left Jaguar", (Jaguar) driveSubsystemRearLeftJaguar);

        driveSubsystemRearLeftEncoder = new Encoder(4, 5, true, EncodingType.k4X);
        LiveWindow.addSensor("Drive Subsystem", "Rear Left Encoder", driveSubsystemRearLeftEncoder);
        driveSubsystemRearLeftEncoder.setDistancePerPulse(0.00177325);
        driveSubsystemRearLeftEncoder.setPIDSourceParameter(PIDSourceParameter.kRate);
        driveSubsystemRearRightJaguar = new Jaguar(3);
        LiveWindow.addActuator("Drive Subsystem", "Rear Right Jaguar", (Jaguar) driveSubsystemRearRightJaguar);

        driveSubsystemRearRightEncoder = new Encoder(6, 7, true, EncodingType.k4X);
        LiveWindow.addSensor("Drive Subsystem", "Rear Right Encoder", driveSubsystemRearRightEncoder);
        driveSubsystemRearRightEncoder.setDistancePerPulse(0.00177325);
        driveSubsystemRearRightEncoder.setPIDSourceParameter(PIDSourceParameter.kRate);
        toteLifterSubsystemRaiseSolenoid = new Solenoid(0, 0);
        LiveWindow.addActuator("Tote Lifter Subsystem", "Raise Solenoid", toteLifterSubsystemRaiseSolenoid);

        toteLifterSubsystemLowerSolenoid = new Solenoid(0, 1);
        LiveWindow.addActuator("Tote Lifter Subsystem", "Lower Solenoid", toteLifterSubsystemLowerSolenoid);

        toteLifterSubsystemGateSolenoid = new Solenoid(0, 2);
        LiveWindow.addActuator("Tote Lifter Subsystem", "Gate Solenoid", toteLifterSubsystemGateSolenoid);

        toteLifterSubsystemEjectorExtendedLimitSwitch = new DigitalInput(10);
        LiveWindow.addSensor("Tote Lifter Subsystem", "Ejector Extended Limit Switch", toteLifterSubsystemEjectorExtendedLimitSwitch);

        toteLifterSubsystemEjectorRetractedLimitSwitch = new DigitalInput(11);
        LiveWindow.addSensor("Tote Lifter Subsystem", "Ejector Retracted Limit Switch", toteLifterSubsystemEjectorRetractedLimitSwitch);

        toteLifterSubsystemEjectorTalon = new Talon(5);
        LiveWindow.addActuator("Tote Lifter Subsystem", "Ejector Talon", (Talon) toteLifterSubsystemEjectorTalon);

        containerHookSubsystemUpperLimitSwitch = new DigitalInput(8);
        LiveWindow.addSensor("Container Hook Subsystem", "Upper Limit Switch", containerHookSubsystemUpperLimitSwitch);

        containerHookSubsystemLowerLimitSwitch = new DigitalInput(9);
        LiveWindow.addSensor("Container Hook Subsystem", "Lower Limit Switch", containerHookSubsystemLowerLimitSwitch);

        containerHookSubsystemTalon = new Talon(4);
        LiveWindow.addActuator("Container Hook Subsystem", "Talon", (Talon) containerHookSubsystemTalon);

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        driveSubsystemGyro = new AnalogGyro(0);
        driveSubsystemAccelerometer = new BuiltInAccelerometer(Range.k8G);

        driveSubsystemFrontLeftWheel = new DIOEncoderWheelController<>(driveSubsystemFrontLeftEncoder, DRIVE_SUBSYSTEM_WHEEL_SPEED_PID_CONSTANTS, DRIVE_SUBSYSTEM_MAX_WHEEL_SPEED, new int[] { 3 }, driveSubsystemFrontLeftJaguar);
        SmartDashboard.putData("Front Left Wheel", driveSubsystemFrontLeftWheel);
        driveSubsystemFrontRightWheel = new DIOEncoderWheelController<>(driveSubsystemFrontRightEncoder, DRIVE_SUBSYSTEM_WHEEL_SPEED_PID_CONSTANTS, DRIVE_SUBSYSTEM_MAX_WHEEL_SPEED, new int[] { 2 }, driveSubsystemFrontRightJaguar);
        SmartDashboard.putData("Front Right Wheel", driveSubsystemFrontRightWheel);
        driveSubsystemRearLeftWheel = new DIOEncoderWheelController<>(driveSubsystemRearLeftEncoder, DRIVE_SUBSYSTEM_WHEEL_SPEED_PID_CONSTANTS, DRIVE_SUBSYSTEM_MAX_WHEEL_SPEED, new int[] { 12 }, driveSubsystemRearLeftJaguar);
        SmartDashboard.putData("Rear Left Wheel", driveSubsystemRearLeftWheel);
        driveSubsystemRearRightWheel = new DIOEncoderWheelController<>(driveSubsystemRearRightEncoder, DRIVE_SUBSYSTEM_WHEEL_SPEED_PID_CONSTANTS, DRIVE_SUBSYSTEM_MAX_WHEEL_SPEED, new int[] { 13 }, driveSubsystemRearRightJaguar);
        SmartDashboard.putData("Rear Right Wheel", driveSubsystemRearRightWheel);
        driveSubsystemDriveController = new FourWheelDriveController<>(driveSubsystemFrontLeftWheel, driveSubsystemFrontRightWheel, driveSubsystemRearLeftWheel, driveSubsystemRearRightWheel);
        driveSubsystemMecanumDriveAlgorithm = new EncoderGyroMecanumDriveAlgorithm<>(driveSubsystemDriveController, RobotMap.driveSubsystemGyro,
                DRIVE_SUBSYSTEM_MECANUM_HEADING_PID_CONSTANTS, DRIVE_SUBSYSTEM_HEADING_TOLERANCE,
                DRIVE_SUBSYSTEM_X_LOCATION_PID_CONSTANTS, DRIVE_SUBSYSTEM_Y_LOCATION_PID_CONSTANTS,
                DRIVE_SUBSYSTEM_LOCATION_TOLERANCE, DRIVE_SUBSYSTEM_LOCATION_TOLERANCE,
                DRIVE_SUBSYSTEM_DRIVE_BASE_WIDTH, DRIVE_SUBSYSTEM_DRIVE_BASE_LENGTH);
        driveSubsystemMecanumDriveAlgorithm.setHeadingInverted(true);
        driveSubsystemArcadeDriveAlgorithm = new ArcadeDriveAlgorithm(driveSubsystemDriveController);

        // ledController = new LEDController(Port.kUSB);
    }
}