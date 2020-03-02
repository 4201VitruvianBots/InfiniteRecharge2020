/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.orchestra.SetSong;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.climber.EnableClimbMode;
import frc.robot.commands.autonomous.*;
import frc.robot.commands.climber.SetClimberOutput;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.intake.ControlledIntake;
import frc.robot.commands.intake.ToggleIntakePistons;
import frc.robot.commands.shooter.*;
import frc.robot.commands.turret.SetTurretSetpointFieldAbsolute;
import frc.robot.commands.turret.ToggleTurretControlMode;
import frc.robot.subsystems.*;
import frc.robot.constants.Constants;
import frc.vitruvianlib.utils.JoystickWrapper;
import frc.vitruvianlib.utils.XBoxTrigger;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final PowerDistributionPanel pdp = new PowerDistributionPanel();

    private final Climber m_climber = new Climber();
//    private final DriveTrain m_driveTrain = new DriveTrain(pdp);
    private final Intake m_intake = new Intake();
//    private final Shooter m_shooter = new Shooter(pdp);
    private final Skyhook m_skyhook = new Skyhook();
//    private final Turret m_turret = new Turret(m_driveTrain);
    private final Vision m_vision = new Vision();
    private final Indexer m_indexer = new Indexer();
    private final LED m_led = new LED();
    private final Orchestra1 m_orchestra = new Orchestra1();
//    private final Controls m_controls = new Controls(m_driveTrain, m_shooter, m_turret, pdp);

    static JoystickWrapper leftJoystick = new JoystickWrapper(Constants.leftJoystick);
    static JoystickWrapper rightJoystick = new JoystickWrapper(Constants.rightJoystick);
    static JoystickWrapper xBoxController = new JoystickWrapper(Constants.xBoxController);
    public Button[] leftButtons = new Button[2];
    public Button[] rightButtons = new Button[2];
    public Button[] xBoxButtons = new Button[10];
    public Button[] xBoxPOVButtons = new Button[8];
    public Button xBoxLeftTrigger, xBoxRightTrigger;

    private static boolean init = false;

    private enum CommandSelector {
        DRIVE_STRAIGHT,
        TEST_PATH,
        FULL_PATH,
        ENEMY_PATH,
        CENTER_PATH,
        debug1,
        debug2,
        TEST_SEQUENTIAL_FORWARD_AUTO,
        TEST_SEQUENTIAL_SWITCHING_AUTO,
        TEST_SEQUENTIAL_REVERSE_AUTO
    }

//    SendableChooser<Integer> m_autoChooser = new SendableChooser();
//
//    private SelectCommand m_autoCommand = new SelectCommand(
//            Map.ofEntries(
//                    entry(CommandSelector.DRIVE_STRAIGHT, new TestPathFollowing(m_driveTrain)),
//                    entry(CommandSelector.TEST_PATH, new TestAuto(m_driveTrain, m_shooter, m_indexer, m_intake)),
//                    entry(CommandSelector.FULL_PATH, new AllyTrenchPath(m_driveTrain, m_shooter, m_indexer, m_intake, m_turret)),
//                    entry(CommandSelector.ENEMY_PATH, new EnemyAutoPath(m_driveTrain, m_shooter, m_indexer, m_intake, m_turret)),
//                    entry(CommandSelector.debug1, new ReadTrajectory(m_driveTrain, "init4Enemy1", true)),
//                    entry(CommandSelector.debug2, new ReadTrajectory(m_driveTrain, "enemy1Shooting1")),
//                    entry(CommandSelector.CENTER_PATH, new CenterAutoPath(m_driveTrain, m_shooter, m_indexer, m_intake, m_turret)),
//                    entry(CommandSelector.TEST_SEQUENTIAL_FORWARD_AUTO, new TestSequentialForward(m_driveTrain)),
//                    entry(CommandSelector.TEST_SEQUENTIAL_SWITCHING_AUTO, new TestSequentialReverse(m_driveTrain)),
//                    entry(CommandSelector.TEST_SEQUENTIAL_REVERSE_AUTO, new TestSequentialSwitching(m_driveTrain))
//            ),
//            this::selectCommand
//    );

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
//        m_autoChooser.addDefault("Drive Straight", CommandSelector.DRIVE_STRAIGHT.ordinal());
//        for (Enum commandEnum : CommandSelector.values())
//            if (commandEnum != CommandSelector.DRIVE_STRAIGHT)
//                m_autoChooser.addOption(commandEnum.toString(), commandEnum.ordinal());
//
//        SmartDashboard.putData(m_autoChooser);

        initializeSubsystems();
        // Configure the button bindings
        configureButtonBindings();
    }

    public void initializeSubsystems() {
//        m_driveTrain.setDefaultCommand(new SetArcadeDrive(m_driveTrain, m_intake,
//                () -> leftJoystick.getRawAxis(1),
//                () -> rightJoystick.getRawAxis(0)));
//
//        m_led.setDefaultCommand(new GetSubsystemStates(this, m_led, m_indexer, m_intake, m_vision, m_turret, m_climber, m_controls));
//
//        m_turret.setDefaultCommand(new SetTurretSetpointFieldAbsolute(m_turret, m_driveTrain, m_vision, m_climber, xBoxController));
//
////    m_shooter.setDefaultCommand(new DefaultFlywheelRPM(m_shooter, m_vision));
//
//        m_climber.setDefaultCommand(new SetClimberOutput(m_climber, xBoxController));
//        m_skyhook.setDefaultCommand(new SetSkyhookOutput(m_climber, m_skyhook, () -> rightJoystick.getRawAxis(0)));
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        leftJoystick.invertRawAxis(1, true);
        rightJoystick.invertRawAxis(0, true);
        xBoxController.invertRawAxis(1, true);
        xBoxController.invertRawAxis(5, true);
        for (int i = 0; i < leftButtons.length; i++)
            leftButtons[i] = new JoystickButton(leftJoystick, (i + 1));
        for (int i = 0; i < rightButtons.length; i++)
            rightButtons[i] = new JoystickButton(rightJoystick, (i + 1));
        for (int i = 0; i < xBoxButtons.length; i++)
            xBoxButtons[i] = new JoystickButton(xBoxController, (i + 1));
        for (int i = 0; i < xBoxPOVButtons.length; i++)
            xBoxPOVButtons[i] = new POVButton(xBoxController, (i * 45));
        xBoxLeftTrigger = new XBoxTrigger(xBoxController, 2);
        xBoxRightTrigger = new XBoxTrigger(xBoxController, 3);

//        leftButtons[0].whileHeld(new SetDriveShifters(m_driveTrain, true));   // Top Button - Switch to high gear
//        leftButtons[1].whileHeld(new SetDriveShifters(m_driveTrain, false));  // Bottom Button - Switch to low gear

//        rightButtons[1].whileHeld(new BrakeWhileHeld(m_driveTrain));
//    rightButtons[0].whileHeld(new AlignToBall(m_driveTrain, m_vision, () -> leftJoystick.getRawAxis(1))); //Bottom (right) Button - Turn to powercells (Automated vision targeting
//    rightButtons[1].whileHeld(new AlignToBall(m_driveTrain, m_vision, () -> leftJoystick.getRawAxis(1))); //Bottom (right) Button - Turn to powercells (Automated vision targeting

//        xBoxButtons[4].whenPressed(new ToggleIntakePistons(m_intake));
//        xBoxLeftTrigger.whileHeld(new ControlledIntake(m_intake, m_indexer)); // Deploy intake
//
//        xBoxButtons[0].whileHeld(new SetRpmSetpoint(m_shooter, m_vision, 3550));                          // A - Set RPM Close
//        xBoxButtons[1].whileHeld(new SetRpmSetpoint(m_shooter, m_vision, 3700));                          // B - Set RPM Medium
//        xBoxButtons[2].whileHeld(new EjectAll(m_indexer, m_intake));                                  // X - Eject All
//        xBoxButtons[3].whileHeld(new SetRpmSetpoint(m_shooter, m_vision, 3900));                          // Y - Set RPM Far

        //xBoxButtons[5].whileHeld(new RapidFire(m_shooter, m_indexer, m_intake, 3700));              // Set Distance RPM
//        xBoxRightTrigger.whileHeld(new RapidFireSetpoint(m_shooter, m_indexer, m_intake));            // flywheel on toggle
//
//        xBoxButtons[6].whenPressed(new ToggleTurretControlMode(m_turret));                            // start - toggle control mode turret
//        //xBoxButtons[7].whenPressed(new ToggleIndexerControlMode(m_indexer));                        // select - toggle control mode uptake
////        //xBoxButtons[8].whenPressed(new Command()); //left stick
////        xBoxButtons[9].whenPressed(new EnableClimbMode(m_climber, m_turret));                         // R3 - toggle driver climb mode?
//
//
//        xBoxPOVButtons[4].whenPressed(new ZeroTurretEncoder(m_turret));
//        //xBoxPOVButtons[4].whileHeld(new EjectAll(m_indexer, m_intake));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

//    private CommandSelector selectCommand() {
////        return CommandSelector.values()[m_autoChooser.getSelected()];
//        return true;
//    }

    public Command getAutonomousCommand() {
        //return m_autoCommand;
        return new WaitCommand(0);
    }

    public void robotPeriodic() {

    }

    public void teleOpInit() {
//        m_driveTrain.resetEncoderCounts();
//        m_driveTrain.resetOdometry(new Pose2d(), new Rotation2d());
    }

    public void teleOpPeriodic() {

    }

    public void autonomousInit() {
    }

    public void autonomousPeriodic() {
    }

    public static void setInitializationState(boolean state) {
        init = state;
    }

    public static boolean getInitializationState() {
        return init;
    }

    public void initalizeLogTopics() {
//        m_controls.initLogging();
    }
}
