/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.LED.GetSubsystemStates;
import frc.robot.commands.autonomous.routines.AccuracyChallenge;
import frc.robot.commands.autonomous.routines.AllyTrenchPathStraight;
import frc.robot.commands.autonomous.routines.EnemyTrenchPath;
import frc.robot.commands.autonomous.routines.GetSOTMTestPowers;
import frc.robot.commands.climber.EnableClimbMode;
import frc.robot.commands.climber.SetClimberOutput;
import frc.robot.commands.drivetrain.AlignToBall;
import frc.robot.commands.drivetrain.BrakeWhileHeld;
import frc.robot.commands.drivetrain.DriveForwardDistance;
import frc.robot.commands.drivetrain.SetArcadeDrive;
import frc.robot.commands.drivetrain.SetDriveShifters;
import frc.robot.commands.indexer.EjectAll;
import frc.robot.commands.indexer.FeedAll;
import frc.robot.commands.intake.ControlledIntake;
import frc.robot.commands.intake.ToggleIntakePistons;
import frc.robot.commands.shooter.RapidFireSetpoint;
import frc.robot.commands.shooter.SetRpmSetpoint;
import frc.robot.commands.shooter.TestAutomatedShooting;
import frc.robot.commands.skyhook.SetSkyhookOutput;
import frc.robot.commands.turret.SetTurretSetpointFieldAbsolute;
import frc.robot.commands.turret.ShootOnTheMove;
import frc.robot.commands.turret.ToggleTurretControlMode;
import frc.robot.commands.turret.ZeroTurretEncoder;
import frc.robot.constants.Constants;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.SimulationShoot;
import frc.robot.subsystems.*;
import frc.vitruvianlib.utils.JoystickWrapper;
import frc.vitruvianlib.utils.XBoxTrigger;

import java.util.Map;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final PowerDistributionPanel pdp = new PowerDistributionPanel();
    private final DriveTrain m_driveTrain = new DriveTrain(pdp);
    private final Intake m_intake = new Intake();
    private final Indexer m_indexer = new Indexer();
    private final Turret m_turret = new Turret(m_driveTrain);
    private final Vision m_vision = new Vision(m_driveTrain, m_turret);
    private final Shooter m_shooter = new Shooter(m_vision, pdp);
    private final Climber m_climber = new Climber();
    private final Skyhook m_skyhook = new Skyhook();
    private final ColorSensor m_colorSensor = new ColorSensor();
    private final LED m_led = new LED(m_colorSensor);
    // private final Controls m_controls = new Controls(m_driveTrain, m_shooter, m_turret, pdp);

    static JoystickWrapper leftJoystick = new JoystickWrapper(Constants.leftJoystick);
    static JoystickWrapper rightJoystick = new JoystickWrapper(Constants.rightJoystick);
    static JoystickWrapper xBoxController = new JoystickWrapper(Constants.xBoxController);
    private ShootOnTheMove m_ShootOnTheMove;
    private final SelectCommand m_autoCommand;
    public Button[] leftButtons = new Button[2];
    public Button[] rightButtons = new Button[2];
    public Button[] xBoxButtons = new Button[10];
    public Button[] xBoxPOVButtons = new Button[8];
    public Button xBoxLeftTrigger, xBoxRightTrigger;

    static JoystickWrapper testController = new JoystickWrapper(4);
    public Button[] testButtons = new Button[10];

    private static boolean init = false;

    private enum CommandSelector {
        DRIVE_STRAIGHT,
        ALLIANCE_TRENCH_STRAIGHT,
        ALLIANCE_TRENCH_SPLINE,
        ENEMY_TRENCH,
        SHOOT_AND_DRIVE_BACK,
        SHOOT_AND_DRIVE_FORWARD,
        DO_NOTHING
    }

    public enum SkillsChallengeSelector {
        ACCURACY_CHALLENGE,
        AUTO_NAV_SLALOM,
        AUTO_NAV_BARREL,
        AUTO_NAV_BOUNCE,
        LIGHSTPEED_CIRCUIT,
        GALACTIC_SEARCH,
        GALACTIC_SEARCH_A,
        GALACTIC_SEARCH_B,
        None
    }

    private SkillsChallengeSelector selectedSkillsChallenge = SkillsChallengeSelector.GALACTIC_SEARCH; // Change this

    private FieldSim m_fieldSim;

    SendableChooser<Integer> m_autoChooser = new SendableChooser();

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_autoChooser.addDefault("Drive Straight", CommandSelector.DRIVE_STRAIGHT.ordinal());
        for(Enum commandEnum : CommandSelector.values())
            if(commandEnum != CommandSelector.DRIVE_STRAIGHT)
                m_autoChooser.addOption(commandEnum.toString(), commandEnum.ordinal());

        SmartDashboard.putData(m_autoChooser);

        GetSOTMTestPowers variableTestPowers = new GetSOTMTestPowers(0.22, 7);
        double startTimestamp = Timer.getFPGATimestamp();

        m_autoCommand = new SelectCommand(
                Map.ofEntries(
                        //entry(CommandSelector.SOTM_STILL, new SOTMWhileStill(m_driveTrain, m_ShootOnTheMove, m_indexer)),
                        //entry(CommandSelector.SOTM_LINE_CONSTANT, new SOTMWhileMovingConstant(m_driveTrain, m_ShootOnTheMove, m_indexer, 0.2, 0.2)), // Replace these values w/ something smart
                        //entry(CommandSelector.SOTM_LINE_VARIABLE, new SOTMWhileMovingVariable(m_driveTrain, m_ShootOnTheMove, m_indexer,
                        //        () -> variableTestPowers.getVariablePower(Timer.getFPGATimestamp() - startTimestamp),
                        //        () -> variableTestPowers.getVariablePower(Timer.getFPGATimestamp() - startTimestamp))),
                        //entry(CommandSelector.SOTM_ARC_CONSTANT, new SOTMWhileMovingConstant(m_driveTrain, m_ShootOnTheMove, m_indexer, 0.21, 0.15)), // Replace these values w/ something smart

                        // entry(CommandSelector.SHOOT_AND_DRIVE_BACK, new ShootAndDriveBack(m_driveTrain, m_intake, m_indexer, m_turret, m_shooter, m_vision)),
                        // entry(CommandSelector.DRIVE_STRAIGHT, new DriveBackwards(m_driveTrain)),
                        // entry(CommandSelector.DO_NOTHING, new DoNothing()),
                        // entry(CommandSelector.SHOOT_AND_DRIVE_FORWARD, new ShootAndDriveForward(m_driveTrain, m_intake, m_indexer, m_turret, m_shooter, m_vision)),
                        // entry(CommandSelector.ALLIANCE_TRENCH_STRAIGHT, new AllyTrenchPathStraight(m_driveTrain, m_intake, m_indexer, m_turret, m_shooter, m_vision))
//                        entry(CommandSelector.TEST_PATH, new TestAuto(m_driveTrain, m_shooter, m_indexer, m_intake)),
//                        entry(CommandSelector.FULL_PATH, new AllyTrenchPathStraight(m_driveTrain, m_intake, m_indexer, m_turret, m_shooter, m_vision))
//                        entry(CommandSelector.ENEMY_PATH, new EnemyAutoPath(m_driveTrain, m_shooter, m_indexer, m_intake, m_turret)),
//                        entry(CommandSelector.debug1, new ReadTrajectoryOld(m_driveTrain, "init4Enemy1", true)),
//                        entry(CommandSelector.debug2, new ReadTrajectoryOld(m_driveTrain, "enemy1Shooting1")),
//                        entry(CommandSelector.CENTER_PATH, new CenterAutoPath(m_driveTrain, m_shooter, m_indexer, m_intake, m_turret)),
//                        entry(CommandSelector.TEST_SEQUENTIAL_FORWARD_AUTO, new TestSequentialForward(m_driveTrain)),
//                        entry(CommandSelector.TEST_SEQUENTIAL_SWITCHING_AUTO, new TestSequentialReverse(m_driveTrain)),
//                        entry(CommandSelector.TEST_SEQUENTIAL_REVERSE_AUTO, new TestSequentialSwitching(m_driveTrain))
                ),
                this :: selectCommand
        );

        initializeSubsystems();
        // Configure the button bindings
        configureButtonBindings();
    }

    public static boolean getInitializationState() {
        return init;
    }

    public static void setInitializationState(boolean state) {
        init = state;
    }

    public void initializeSubsystems() {
//        for(int i = 0; i < 6; i++)
//            m_powercells[i] = new Powercell("PowerCell_" + i);

        m_fieldSim = new FieldSim(m_driveTrain, m_turret, m_shooter);

        if(RobotBase.isReal()) {
            m_driveTrain.setDefaultCommand(new SequentialCommandGroup(new SetDriveShifters(m_driveTrain, Constants.DriveConstants.inSlowGear), 
            new SetArcadeDrive(m_driveTrain, m_intake,
                    () -> -leftJoystick.getRawAxis(1),
                    () -> rightJoystick.getRawAxis(0))));

            m_led.setDefaultCommand(new GetSubsystemStates(this, m_led, m_indexer, m_intake, m_vision, m_turret, m_climber, m_colorSensor));
        }
        else {
            m_fieldSim.placeSkillPowercells(selectedSkillsChallenge);
            m_driveTrain.setDefaultCommand(new SetArcadeDrive(m_driveTrain, m_intake,
                    () -> testController.getRawAxis(1),
                    () -> testController.getRawAxis(2)));
        }
        m_turret.setDefaultCommand(new SetTurretSetpointFieldAbsolute(m_turret, m_driveTrain, m_vision, m_shooter, m_climber, xBoxController));

//    m_shooter.setDefaultCommand(new DefaultFlywheelRPM(m_shooter, m_vision));

        m_climber.setDefaultCommand(new SetClimberOutput(m_climber, xBoxController));
        m_skyhook.setDefaultCommand(new SetSkyhookOutput(m_climber, m_skyhook, () -> rightJoystick.getRawAxis(0)));
        m_ShootOnTheMove = new ShootOnTheMove(m_turret, m_shooter, m_driveTrain, m_led, m_vision, new SimulationShoot(m_fieldSim, true),  m_fieldSim);
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        if(RobotBase.isReal()) {
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
            if (selectedSkillsChallenge == SkillsChallengeSelector.ACCURACY_CHALLENGE) {
                configGameChangersButtons();
            } else {
                configInfiniteRechargeButtons();
            }
        }
        else {
            testController.invertRawAxis(1, true);
            testController.invertRawAxis(5, true);
            for (int i = 0; i < testButtons.length; i++)
                testButtons[i] = new JoystickButton(testController, (i + 1));
            //testButtons[0].whenPressed(new SimulationShoot(m_FieldSim, false));
//            testButtons[3].toggleWhenPressed(m_ShootOnTheMove); // Y - Shoot on the Move
            testButtons[0].whileHeld(new FeedAll(m_indexer));
        }
    }

    private void configInfiniteRechargeButtons() {
        leftButtons[0].whileHeld(new SetDriveShifters(m_driveTrain, false));   // Top Button - Switch to high gear
        leftButtons[1].whileHeld(new SetDriveShifters(m_driveTrain, true));  // Bottom Button - Switch to low gear

        rightButtons[0].whileHeld(new AlignToBall(m_driveTrain, m_vision,
                () -> -leftJoystick.getRawAxis(1),
                () -> rightJoystick.getRawAxis(0))); //Bottom (right) Button - Turn to powercells (Automated vision targeting
        rightButtons[1].whileHeld(new BrakeWhileHeld(m_driveTrain));
        
        xBoxButtons[4].whenPressed(new ToggleIntakePistons(m_intake));
        xBoxLeftTrigger.whileHeld(new ControlledIntake(m_intake, m_indexer, xBoxController)); // Deploy intake

        xBoxButtons[0].whileHeld(new SetRpmSetpoint(m_shooter, m_vision, 1000)); // [A] Short-range
        xBoxButtons[1].whileHeld(new SetRpmSetpoint(m_shooter, m_vision, 3410)); // [B] Med-range
        xBoxButtons[3].whileHeld(new SetRpmSetpoint(m_shooter, m_vision, 3900)); // [Y] Long-range
        xBoxPOVButtons[0].whileHeld(new EjectAll(m_indexer, m_intake));                                  //Top POV - Eject All
        xBoxButtons[0].whileHeld(new TestAutomatedShooting(m_driveTrain, m_shooter, m_turret, m_vision));

        //xBoxButtons[5].whileHeld(new RapidFire(m_shooter, m_indexer, m_intake, 3700));              // Set Distance RPM
        xBoxRightTrigger.whileHeld(new RapidFireSetpoint(m_shooter, m_indexer, m_intake));            // flywheel on toggle

        xBoxButtons[6].whenPressed(new ToggleTurretControlMode(m_turret));                            // start - toggle control mode turret
        //xBoxButtons[7].whenPressed(new ToggleIndexerControlMode(m_indexer));                        // select - toggle control mode uptake
//        xBoxButtons[8].whenPressed(new DisableClimbMode(m_climber,m_turret)); //left stick
        xBoxButtons[9].whenPressed(new EnableClimbMode(m_climber, m_turret));                         // R3 - toggle driver climb mode?

        xBoxPOVButtons[4].whenPressed(new ZeroTurretEncoder(m_turret));
        //xBoxPOVButtons[4].whileHeld(new EjectAll(m_indexer, m_intake));
//        SmartDashboard.putData("disable climb mode", new DisableClimbMode(m_climber,m_turret));
    }

    private void configGameChangersButtons() {
        xBoxButtons[4].whenPressed(new ToggleIntakePistons(m_intake));
        xBoxLeftTrigger.whileHeld(new ControlledIntake(m_intake, m_indexer, xBoxController)); // Deploy intake

        xBoxButtons[1].whenPressed(new AccuracyChallenge(m_driveTrain, m_shooter, m_indexer, m_fieldSim, 0));      // B - Green Zone
        xBoxButtons[2].whenPressed(new AccuracyChallenge(m_driveTrain, m_shooter, m_indexer, m_fieldSim, 1));      // X - Yellow Zone
        xBoxButtons[3].whenPressed(new AccuracyChallenge(m_driveTrain, m_shooter, m_indexer, m_fieldSim, 2));      // Y - Blue Zone
        xBoxButtons[0].whenPressed(new AccuracyChallenge(m_driveTrain, m_shooter, m_indexer, m_fieldSim, 3));      // A - Red Zone
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    private CommandSelector selectCommand() {
        return CommandSelector.values()[m_autoChooser.getSelected()];
    }

    public Command getAutonomousCommand() {
        // return new DriveForwardDistance(m_driveTrain, m_fieldSim, 2.5);
       return new AllyTrenchPathStraight(m_driveTrain, m_intake, m_indexer, m_turret, m_shooter, m_vision);
        // return new EnemyTrenchPath(m_driveTrain, m_intake, m_indexer, m_turret, m_shooter, m_vision, m_fieldSim);
        /*switch (selectedSkillsChallenge) {
            case AUTO_NAV_BARREL:
                return new AutoNavBarrel(m_driveTrain, m_FieldSim);
            case AUTO_NAV_BOUNCE:
                return new AutoNavBounce(m_driveTrain, m_FieldSim);
            case AUTO_NAV_SLALOM:
                return new AutoNavSlalom(m_driveTrain, m_FieldSim);
            case LIGHSTPEED_CIRCUIT:
                return new LightspeedCircuit(m_driveTrain, m_FieldSim);
            case GALACTIC_SEARCH:
                return new SequentialCommandGroup(
                    //new SetIntakePiston(m_intake, true), 
                    new GalacticSearch(m_driveTrain, m_FieldSim).deadlineWith(new AutoControlledIntake(m_intake, m_indexer)),
                    new SetIntakePiston(m_intake, false));
            case GALACTIC_SEARCH_A:
                return new SequentialCommandGroup(
                    //new SetIntakePiston(m_intake, true), 
                    new GalacticSearchA(m_driveTrain, m_FieldSim).deadlineWith(new AutoControlledIntake(m_intake, m_indexer)),
                    //(new ConditionalCommand(new GalacticSearchARed(m_driveTrain, m_FieldSim), new GalacticSearchABlue(m_driveTrain, m_FieldSim), () -> true)).deadlineWith(new AutoControlledIntake(m_intake, m_indexer)),
                    new SetIntakePiston(m_intake, false));
            case GALACTIC_SEARCH_B:
                return new SequentialCommandGroup(
                    // new SetIntakePiston(m_intake, true), 
                    new GalacticSearchB(m_driveTrain, m_FieldSim).deadlineWith(new AutoControlledIntake(m_intake, m_indexer)),
                    new SetIntakePiston(m_intake, false));
            case None:
            default:
                System.out.println("Not a recognized skills command");
                return null;
        }*/
        
//            return new SOTMSimulationAuto(m_driveTrain, m_intake, m_indexer, m_turret, m_shooter, m_vision, m_FieldSim, m_ShootOnTheMove);
            //return m_ShootOnTheMove;
//            return new AllyTrenchPathStraight(m_driveTrain, m_intake, m_indexer, m_turret, m_shooter, m_vision);
//            return new AllyTrenchPathSplineSim(m_driveTrain, m_intake, m_indexer, m_turret, m_shooter, m_vision, m_FieldSim);
            //return new OpRoutineRed(m_driveTrain, m_intake, m_indexer, m_turret, m_shooter, m_vision, m_FieldSim);
//            return new DriveStraight(m_driveTrain, m_turret, m_FieldSim);
//        return new WaitCommand(0);
    }

    public void disabledInit() {
        setInitializationState(true);
        m_driveTrain.setDriveTrainNeutralMode(2);
    }

    public void robotPeriodic() {

    }

    public void teleOpInit() {
        if(RobotBase.isReal()) {
            m_driveTrain.resetEncoderCounts();
            m_driveTrain.resetOdometry(m_fieldSim.getRobotPose(), m_fieldSim.getRobotPose().getRotation());
            m_driveTrain.setDriveTrainNeutralMode(0); // Half and half
        } else {
            m_driveTrain.resetEncoderCounts();
            m_driveTrain.resetOdometry(m_fieldSim.getRobotPose(), m_fieldSim.getRobotPose().getRotation());
        }
    }

    public void teleOpPeriodic() {

    }

    public void autonomousInit() {
        if (RobotBase.isReal()) {
            m_driveTrain.resetEncoderCounts();
            m_driveTrain.resetOdometry(m_driveTrain.getRobotPose(), m_fieldSim.getRobotPose().getRotation());
        } else {
            m_fieldSim.initSim();
            m_driveTrain.resetEncoderCounts();
            m_driveTrain.resetOdometry(m_fieldSim.getRobotPose(), m_fieldSim.getRobotPose().getRotation());
        }
    }

    public void autonomousPeriodic() {
    }

    public void initializeLogTopics() {
        //m_controls.initLogging();
    }

    /*private enum CommandSelector {
        DRIVE_STRAIGHT,
        ALLIANCE_TRENCH_STRAIGHT,
        ALLIANCE_TRENCH_SPLINE,
        ENEMY_TRENCH,
        SHOOT_AND_DRIVE_BACK,
        SHOOT_AND_DRIVE_FORWARD,
        DO_NOTHING,
        SOTM_STILL,
        SOTM_LINE_CONSTANT,
        SOTM_LINE_VARIABLE,
        SOTM_ARC_CONSTANT
    }*/

    public void initalizeLogTopics() {
        //m_controls.initLogging();
    }

    public DriveTrain getRobotDrive() {
        return m_driveTrain;
    }

    public void simulationInit() {
        m_fieldSim.initSim();
        //m_driveTrain.setSimPose(new Pose2d(5,5, new Rotation2d()));
    }

    public void simulationPeriodic() {
        if(!RobotState.isTest())
            m_fieldSim.simulationPeriodic();
    }

    public void ledPeriodic(){
        m_led.ledPeriodic();
    }
}
