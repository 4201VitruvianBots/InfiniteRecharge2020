package frc.robot.simulation;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class SimConstants {
    public static final double fieldWidth = 15.980;
    public static final double fieldHieght = 8.210;
    public static final double ballDiameter = 0.1778; // In meters

    public static final double robotWidth = 0.673;
    public static final double robotLength = 0.838;
    public static final double intakeLength = 0.3048;
    public static final double shotSpeed = 10; // in meters/second;

    public static final Pose2d redLoadingStation = new Pose2d(0.238258, 2.554548, new Rotation2d());
    public static final Pose2d blueLoadingStation = new Pose2d(15.732665, 5.646024, new Rotation2d());

    public static final Pose2d[] blueTrenchBallPos = {
            new Pose2d(6.154554,7.506032, new Rotation2d()),
            new Pose2d(7.064754,7.506032, new Rotation2d()),
            new Pose2d(7.993157,7.506032, new Rotation2d()),
            new Pose2d(9.615867,7.275692, new Rotation2d()),
            new Pose2d(9.615867,7.744320, new Rotation2d())
    };

    public static final Pose2d[] blueCenterBalls = {
            new Pose2d(0,0, new Rotation2d()),
            new Pose2d(0,0, new Rotation2d()),
            new Pose2d(0,0, new Rotation2d()),
            new Pose2d(6.340368,5.299097, new Rotation2d()),
            new Pose2d(5.943837,5.118855, new Rotation2d()),
    };

    public static final Pose2d[] redTrenchBallPos = {
            new Pose2d(9.814133,0.660829, new Rotation2d()),
            new Pose2d(8.894900,0.660829, new Rotation2d()),
            new Pose2d(7.975669,0.660829, new Rotation2d()),
            new Pose2d(6.340368,0.919229, new Rotation2d()),
            new Pose2d(6.340368,0.432577, new Rotation2d()),
    };

    public static final Pose2d[] redCenterBalls = {
            new Pose2d(0,0, new Rotation2d()),
            new Pose2d(0,0, new Rotation2d()),
            new Pose2d(0,0, new Rotation2d()),
            new Pose2d(10.012398,3.040016, new Rotation2d()),
            new Pose2d(9.615867,2.877799, new Rotation2d()),
    };

    public static final Pose2d blueGoalPose = new Pose2d(0, 5.831, new Rotation2d());
    public static final Pose2d redGoalPose = new Pose2d(15.960367,2.373122, new Rotation2d());
}
