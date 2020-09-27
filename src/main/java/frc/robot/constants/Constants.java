/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.constants;

import edu.wpi.first.wpilibj.util.Units;

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

    // Shoot on the Move
    public static final double g = 9.81; // Absolute value, in meters per second squared
    public static final double airResistanceCoefficient = 1.05; // Constant that shoot velocity is multiplied by to account for air resistance

    // Target measurements
    public static final double ballRadius = Units.inchesToMeters(3.5);
    public static final double ballTolerance = Units.inchesToMeters(1);
    public static final double outerTargetHeight = Units.inchesToMeters(30);
    public static final double targetOffset = Units.inchesToMeters(29.25);

    // Use actual values for these
    public static final double verticalTargetDistance = Units.inchesToMeters(98.25 - 38); // Distance between shooter and target heights from ground
    public static final double verticalShooterAngle = 1.05; // Angle ball is shot from shooter relative to the ground 
    public static final double targetXPosition = 0;
    public static final double targetYPosition = Units.inchesToMeters(629.25);

    public static final double turretAcceleration = .75; // radians per second
    public static final double shooterAcceleration = 1000; // RPM per second

    public static final double navXToShooterDistance = Units.inchesToMeters(1.4); // Meters
    public static final double navXToShooterAngle = 0; // Radians; angle offset between navX and shooter

    public static final double maxShooterRPM = 10_000; // Highest RPM that shooter can launch balls at without breaking
}
