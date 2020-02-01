/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static boolean canSeeVisionTarget = false;
    public static boolean limelightTempDisabled = false;

    // USB PORTS

    public static final int leftJoystick = 0;
    public static final int rightJoystick = 1;
    public static final int xBoxController = 2;

    // CAN ADDRESSES
    public static final int leftFrontDriveMotor = 20;
    public static final int leftRearDriveMotor = 21;
    public static final int rightFrontDriveMotor = 22;
    public static final int rightRearDriveMotor = 23;
    public static final int intakeMotor = 40;
    public static final int intakeMotor1 = 41;
    public static final int indexerMotor = 35;
    public static final int flywheelMotorA = 40; // 40
    public static final int flywheelMotorB = 41;
    public static final int leftClimberMotor = 50;
    public static final int rightClimberMotor = 51;
    public static final int skyhookMotor = 55;
    public static final int turretMotor = 60;
    public static final int turretEncoder = 61;

    // DIO
    public static final int indexSensor = 0;
    public static final int indexLimitSensor = 1;

    // PCM
    public static final int pcmOne = 11;

    //Solenoid addresses
    public static final int driveTrainShiftersForward = 0;
    public static final int driveTrainShiftersReverse = 1;
    public static final int intakePistonForward = 2;
    public static final int intakePistonReverse = 3;
}
