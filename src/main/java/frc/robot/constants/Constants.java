/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.constants;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.numbers.N2;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // USB PORTS
    public static final int leftJoystick = 0;
    public static final int rightJoystick = 1;
    public static final int xBoxController = 2;

    // CAN ADDRESSES
    public static final int pcmOne = 11;
    
    public static final int leftFrontDriveMotor = 20;
    public static final int leftRearDriveMotor = 21;
    public static final int rightFrontDriveMotor = 22;
    public static final int rightRearDriveMotor = 23;
    public static final int intakeMotor = 30;
    public static final int indexerMotor = 35;
    public static final int kickerMotor = 36;
    public static final int flywheelMotorA = 40;
    public static final int flywheelMotorB = 41;
    public static final int colorWheelMotor = 45;
    public static final int climbMotorA = 50;
    public static final int climbMotorB = 51;
    public static final int skyhookMotor = 55;
    public static final int turretMotor = 60;
    public static final int turretEncoder = 61;

    // PWM
    public static final int ledPort = 0;

    // DIO
    public static final int intakeSensor = 0;
    public static final int indexerTopSensor = 1;
    public static final int indexerBottomSensor = 2;
    public static final int turretHomeSensor = 3;


    //Solenoid addresses
    public static final int driveTrainShiftersForward = 0;
    public static final int driveTrainShiftersReverse = 1;
    public static final int intakePistonForward = 2; // 2
    public static final int intakePistonReverse = 3; // 3
    public static final int climbPistonForward = 4;
    public static final int climbPistonReverse = 5;

    public static final class DriveConstants {
        public static final int[] kLeftEncoderPorts = new int[]{10, 11};
        public static final int[] kRightEncoderPorts = new int[]{12, 13};
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;

        public static final double kTrackwidthMeters = Units.inchesToMeters(21.5);
        public static final DifferentialDriveKinematics kDriveKinematics =
                new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final int kEncoderCPR = 4096;
        public static final double kWheelDiameterMeters = Units.feetToMeters(0.5);
        public static final double kEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

        public static final boolean kGyroReversed = true;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.19;
        public static final double kvVoltSecondsPerMeter = 2.23;
        public static final double kaVoltSecondsSquaredPerMeter = 0.0289;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // These two values are "angular" kV and kA
        public static final double kvVoltSecondsPerRadian = 1.5;
        public static final double kaVoltSecondsSquaredPerRadian = 0.3;

        public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
                LinearSystemId.identifyDrivetrainSystem(kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter,
                        kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian);

        // Example values only -- use what's on your physical robot!
        public static final DCMotor kDriveGearbox = DCMotor.getFalcon500(2);
        public static final double kDriveGearing = 14.14;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 0.1;
    }
}
