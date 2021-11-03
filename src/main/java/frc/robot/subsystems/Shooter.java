/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import frc.robot.constants.Constants;

/*
Subsystem for controlling to robot's shooter
 */

public class Shooter extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     *
     * @return
     */
    // PID loop constants
    private final double kF = 0.0523;  // 0.054      //  Gree: 0.0475;
    private final double kP = 0.25;      //  0.4       //  0.00047
    private final double kI = 0.00008;                    //  0.0000287
    private final double kD = 7;

//    private double kF = 0.0523;  // 0.054      //  Gree: 0.0475;
//    private double kP = 0.6;      //  0.4       //  0.00047
//    private double kI = 0.0;                    //  0.0000287
//    private double kD = 0.0;

    private final double kS = 0.155;
    private final double kV = 0.111;
    private final double kA = 0.02;

    //    private double kP = 1.91;
//    private double kI = 0.0;
//    private double kD = 0.0;
    private final TalonFX[] outtakeMotors = {
            new TalonFX(Constants.flywheelMotorA),
            new TalonFX(Constants.flywheelMotorB),
    };
    private final PowerDistributionPanel m_pdp;
    private final Vision m_vision;
    public int kI_Zone = 400;
    public int kAllowableError = 50;
    public double rpmOutput;
    public double rpmTolerance = 50.0;
    private double setpoint;
    private boolean timerStart;
    private double timestamp;
    private boolean canShoot;
    private double idealRPM; // The RPM the shooter should be set to in order to hit the target from this distance

//    public PIDController flywheelController = new PIDController(kP, kI, kD);
//    public SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    private final double metersPerSecondToRPM = 315; // How much to multiply a speed by to get ideal RPM. This is only a preliminary estimate

    private final LinearSystem<N1, N1, N1> m_flywheelPlant =
            LinearSystemId.identifyVelocitySystem(kV, kA);

    private final KalmanFilter<N1, N1, N1> m_observer =
            new KalmanFilter<>(
                    Nat.N1(),
                    Nat.N1(),
                    m_flywheelPlant,
                    VecBuilder.fill(3.0), // How accurate we think our model is
                    VecBuilder.fill(0.01), // How accurate we think our encoder
                    // data is
                    0.020);

    private final LinearQuadraticRegulator<N1, N1, N1> m_controller =
            new LinearQuadraticRegulator<>(
                    m_flywheelPlant,
                    VecBuilder.fill(8.0), // Velocity error tolerance
                    VecBuilder.fill(12.0), // Control effort (voltage) tolerance
                    0.020);

    private final LinearSystemLoop<N1, N1, N1> m_loop =
            new LinearSystemLoop<>(m_flywheelPlant, m_controller, m_observer, 12.0, 0.020);

    public Shooter(Vision vision, PowerDistributionPanel pdp) {
        // Setup shooter motors (Falcons)
        for(TalonFX outtakeMotor : outtakeMotors) {
            outtakeMotor.configFactoryDefault();
            outtakeMotor.setNeutralMode(NeutralMode.Coast);
            outtakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0));
            outtakeMotor.configVoltageCompSaturation(10);
            outtakeMotor.enableVoltageCompensation(true);
        }
        outtakeMotors[0].setInverted(true);
        outtakeMotors[1].follow(outtakeMotors[0], FollowerType.PercentOutput);

        outtakeMotors[0].config_kF(0, kF);
        outtakeMotors[0].config_kP(0, kP);
        outtakeMotors[0].config_kI(0, kI);
        outtakeMotors[0].config_IntegralZone(0, kI_Zone);
        outtakeMotors[0].config_kD(0, kD);
        outtakeMotors[0].configAllowableClosedloopError(0, kAllowableError);
        outtakeMotors[0].configClosedloopRamp(0.2);
        outtakeMotors[1].configClosedloopRamp(0);
        outtakeMotors[1].configOpenloopRamp(0);

        m_vision = vision;
        m_pdp = pdp;

        initShuffleboard();
    }

    //Self-explanatory commands

    public double getMotorInputCurrent(int motorIndex) {
        return outtakeMotors[motorIndex].getSupplyCurrent();
    }

    public void setPower(double output) {
        outtakeMotors[0].set(ControlMode.PercentOutput, output);
    }

    public void setRPM(double setpoint) {
        this.setpoint = setpoint;
    }

//    private void updatePidRPM() {
//        if (setpoint >= 0)
//            outtakeMotors[0].set(ControlMode.Velocity, setpoint, DemandType.ArbitraryFeedForward,
//                    feedforward.calculate(setpoint / 60.0));
//        else
//            setPower(0);
//    }

    public double getSetpoint() {
        return setpoint;
    }

    public boolean canShoot() {
        return canShoot;
    }

    private void updateRPMSetpoint() {
        if(setpoint >= 0) {
//            outtakeMotors[0].set(ControlMode.Velocity, RPMtoFalconUnits(setpoint));
            m_loop.setNextR(VecBuilder.fill(setpoint * 2 * Math.PI));

            m_loop.correct(VecBuilder.fill(getAngularVelocity(0));
            m_loop.predict(0.020);

            double nextVoltage = m_loop.getU(0);
            setPower(nextVoltage / 12);
        } else
            setPower(0);
    }

    public void setTestRPM() {
        outtakeMotors[0].set(ControlMode.Velocity, RPMtoFalconUnits(rpmOutput));
    }

    public double getTestRPM() {
        return rpmOutput;
    }

    public double getRPMTolerance() {
        return rpmTolerance;
    }

    public boolean encoderAtSetpoint(int motorIndex) {
        return (Math.abs(outtakeMotors[motorIndex].getClosedLoopError()) < 100.0);
    }

    public double getAngularVelocity(int motorIndex) {
        return outtakeMotors[motorIndex].getSelectedSensorVelocity() * ((600.0 * 2 * Math.PI)/ 2048.0);
    }

    public double getRPM(int motorIndex) {
        return falconUnitsToRPM(outtakeMotors[motorIndex].getSelectedSensorVelocity());
    }

    public double falconUnitsToRPM(double sensorUnits) {
        return (sensorUnits / 2048.0) * 600.0;
    }

    public double RPMtoFalconUnits(double RPM) {
        return (RPM / 600.0) * 2048.0;
    }

    /*
    * Calculates RPM to hit target at a specific distance from the target, based on physics
    * @param distance Distance from target (in meters)
    */
    private void calculateIdealRPM(double distance) {
        double shootSpeed = distance * Math.sqrt(0.5 * Constants.g / (distance * Math.tan(Constants.verticalShooterAngle) - Constants.verticalTargetDistance)) / Math.cos(Constants.verticalShooterAngle);
        idealRPM = shootSpeed * metersPerSecondToRPM;
    }

    public double getIdealRPM(double distance) {
        return (distance * Math.sqrt(0.5 * Constants.g / (distance * Math.tan(Constants.verticalShooterAngle) - Constants.verticalTargetDistance)) / Math.cos(Constants.verticalShooterAngle))
        * metersPerSecondToRPM;
    }

    public void setIdealRPM() {
        setpoint = idealRPM;
    }

    private void initShuffleboard() {
        // Unstable. Don''t use until WPILib fixes this
//    Shuffleboard.getTab("Shooter").addNumber("RPM Primary", () -> this.getRPM(0));
//    Shuffleboard.getTab("Shooter").addNumber("RPM Secondary", () -> this.getRPM(1));
//    Shuffleboard.getTab("Shooter").addNumber("Power", () -> this.outtakeMotors[0].getMotorOutputPercent());

        SmartDashboardTab.putNumber("Shooter", "RPM Output", rpmOutput);
        SmartDashboardTab.putNumber("Shooter", "Flywheel kF", kF);
        SmartDashboardTab.putNumber("Shooter", "Flywheel kP", kP);
        SmartDashboardTab.putNumber("Shooter", "Flywheel kI", kI);
        SmartDashboardTab.putNumber("Shooter", "Flywheel kD", kD);
        SmartDashboardTab.putNumber("Shooter", "Flywheel kI_Zone", kI_Zone);
        SmartDashboardTab.putNumber("Shooter", "Flywheel kAllowableError", kAllowableError);
        SmartDashboardTab.putNumber("Shooter", "Flywheel RPM Tolerance", rpmTolerance);
    }

    private void updateShuffleboard() {
        if (RobotBase.isReal()) {
            SmartDashboard.putNumber("RPM", falconUnitsToRPM(outtakeMotors[0].getSelectedSensorVelocity()));

        SmartDashboardTab.putNumber("Shooter", "RPM Primary", getRPM(0));
        SmartDashboardTab.putNumber("Shooter", "RPM Secondary", getRPM(1));
        SmartDashboardTab.putNumber("Shooter", "Setpoint", setpoint);
        SmartDashboardTab.putNumber("Shooter", "Power", outtakeMotors[0].getMotorOutputPercent());
        SmartDashboardTab.putNumber("Shooter", "Error", getSetpoint() - getRPM(0));

            SmartDashboardTab.putBoolean("DriveTrain", "CanShoot", canShoot());
        }
    }

    public void updatePIDValues() {
        // Allow PID values to be set through SmartDashboard
        rpmOutput = SmartDashboardTab.getNumber("Shooter", "RPM Output", 0);
        rpmTolerance = SmartDashboardTab.getNumber("Shooter", "Flywheel RPM Tolerance", 0);

        outtakeMotors[0].config_kF(0, SmartDashboardTab.getNumber("Shooter", "Flywheel kF", 0));
        outtakeMotors[0].config_kP(0, SmartDashboardTab.getNumber("Shooter", "Flywheel kP", 0));
        outtakeMotors[0].config_kI(0, SmartDashboardTab.getNumber("Shooter", "Flywheel kI", 0));
        outtakeMotors[0].config_IntegralZone(0, (int) SmartDashboardTab.getNumber("Shooter", "Flywheel kI_Zone", 0));
        outtakeMotors[0].config_kD(0, SmartDashboardTab.getNumber("Shooter", "Flywheel kD", 0));
        outtakeMotors[0].configAllowableClosedloopError(0, (int) SmartDashboardTab.getNumber("Shooter", "Flywheel kAllowableError", 0));
    }


    @Override
    public void periodic() {
        updateRPMSetpoint();
//        updatePidRPM();
        updateShuffleboard();
        updatePIDValues();
        //calculateIdealRPM();

        if((Math.abs(getSetpoint() - getRPM(0)) < getRPMTolerance()) && m_vision.hasTarget() &&
                (Math.abs(m_vision.getTargetX()) < 1) && ! timerStart) {
            timerStart = true;
            timestamp = Timer.getFPGATimestamp();
        } else if(((Math.abs(getSetpoint() - getRPM(0)) > getRPMTolerance()) || ! m_vision.hasTarget() ||
                (Math.abs(m_vision.getTargetX()) > 1)) && timerStart) {
            timestamp = 0;
            timerStart = false;
        }
        // canShoot = Math.abs(Timer.getFPGATimestamp() - timestamp) > 0.6 & timestamp != 0;
        if(timestamp != 0) {

            canShoot = Math.abs(Timer.getFPGATimestamp() - timestamp) > 0.6;

        } else
            canShoot = false;
    }
}
