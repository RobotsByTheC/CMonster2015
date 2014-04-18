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

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.PIDSource.PIDSourceParameter;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static SpeedController driveSubsystemFrontLeftJaguar;
    public static SpeedController driveSubsystemFrontRightJaguar;
    public static SpeedController driveSubsystemRearLeftJaguar;
    public static SpeedController driveSubsystemRearRightJaguar;
    public static Encoder driveSubsystemRearRightEncoder;
    public static Compressor compressorSubsystemCompressor;
    public static Solenoid sweeperSubsystemSolenoid;
    public static SpeedController sweeperSubsystemJaguar;
    public static Solenoid catcherSubsytemSolenoid;
    public static DigitalOutput ledSubsystemPin0;
    public static DigitalOutput ledSubsystemPin1;
    public static DigitalOutput ledSubsystemPin2;
    public static DigitalOutput ledSubsystemPin3;
    public static DigitalOutput ledSubsystemPin4;
    public static DigitalOutput ledSubsystemPin5;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static BetterGyro driveSubsystemSteeringGyro;
    public static TempSensor driveSubsystemSteeringGyroTemp;
    public static ADXL345_I2C driveSubsystemAccelerometer;

    public static void init() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        driveSubsystemFrontLeftJaguar = new Jaguar(1, 1);
        LiveWindow.addActuator("Drive Subsystem", "Front Left Jaguar", (Jaguar) driveSubsystemFrontLeftJaguar);

        driveSubsystemFrontRightJaguar = new Jaguar(1, 2);
        LiveWindow.addActuator("Drive Subsystem", "Front Right Jaguar", (Jaguar) driveSubsystemFrontRightJaguar);

        driveSubsystemRearLeftJaguar = new Jaguar(1, 3);
        LiveWindow.addActuator("Drive Subsystem", "Rear Left Jaguar", (Jaguar) driveSubsystemRearLeftJaguar);

        driveSubsystemRearRightJaguar = new Jaguar(1, 4);
        LiveWindow.addActuator("Drive Subsystem", "Rear Right Jaguar", (Jaguar) driveSubsystemRearRightJaguar);

        driveSubsystemRearRightEncoder = new Encoder(1, 2, 1, 3, false, EncodingType.k2X);
        LiveWindow.addSensor("Drive Subsystem", "Rear Right Encoder", driveSubsystemRearRightEncoder);
        driveSubsystemRearRightEncoder.setDistancePerPulse(0.002908882);
        driveSubsystemRearRightEncoder.setPIDSourceParameter(PIDSourceParameter.kDistance);
        driveSubsystemRearRightEncoder.start();
        compressorSubsystemCompressor = new Compressor(1, 1, 1, 1);

        sweeperSubsystemSolenoid = new Solenoid(1, 2);
        LiveWindow.addActuator("Sweeper Subsystem", "Solenoid", sweeperSubsystemSolenoid);

        sweeperSubsystemJaguar = new Jaguar(1, 5);
        LiveWindow.addActuator("Sweeper Subsystem", "Jaguar", (Jaguar) sweeperSubsystemJaguar);

        catcherSubsytemSolenoid = new Solenoid(1, 1);
        LiveWindow.addActuator("Catcher Subsytem", "Solenoid", catcherSubsytemSolenoid);

        ledSubsystemPin0 = new DigitalOutput(1, 4);

        ledSubsystemPin1 = new DigitalOutput(1, 5);

        ledSubsystemPin2 = new DigitalOutput(1, 6);

        ledSubsystemPin3 = new DigitalOutput(1, 7);

        ledSubsystemPin4 = new DigitalOutput(1, 8);

        ledSubsystemPin5 = new DigitalOutput(1, 9);

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        driveSubsystemSteeringGyro = new BetterGyro(1, 1);
        driveSubsystemSteeringGyroTemp = new TempSensor(2);
        driveSubsystemAccelerometer = new ADXL345_I2C(1, ADXL345_I2C.DataFormat_Range.k4G);
    }
}
