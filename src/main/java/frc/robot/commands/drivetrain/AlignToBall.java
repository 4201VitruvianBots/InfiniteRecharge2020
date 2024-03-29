/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

import java.util.function.DoubleSupplier;

public class AlignToBall extends CommandBase {

    private final double P_TERM = 0.02;
    private final double I_TERM = 0;
    private final double D_TERM = 0;

    private final DriveTrain m_driveTrain;
    private final Vision m_vision;
    private final DoubleSupplier m_throttle;
    private final DoubleSupplier m_turn;
    private final PIDController pid = new PIDController(P_TERM, I_TERM, D_TERM);

    public AlignToBall(DriveTrain driveTrain, Vision vision, DoubleSupplier throttle, DoubleSupplier turn) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.m_driveTrain = driveTrain;
        this.m_vision = vision;
        this.m_throttle = throttle;
        this.m_turn = turn;
        addRequirements(this.m_driveTrain);
        addRequirements(this.m_vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double joystickY = (Math.abs(m_throttle.getAsDouble()) > 0.05) ? m_throttle.getAsDouble() : 0;
        double joystickX = (Math.abs(m_turn.getAsDouble()) > 0.05) ? m_turn.getAsDouble() : 0;

        double throttle = joystickY;
        throttle = throttle < 0 ? Math.max(- 0.7, throttle) : throttle;
//        double turn = /*(m_driveTrain.getDriveShifterStatus() ? 0.5 : 0.50.35) **/ joystickX;
        double turn = 0;
        if(m_vision.hasPowerCell()) {
            double setpoint = m_driveTrain.getAngle() + m_vision.getPowerCellX();

            double turnAdjustment = pid.calculate(m_driveTrain.getAngle(), setpoint);

            turn += turnAdjustment;
//            turn += Math.max(Math.min(turnAdjustment, 0.6), -0.6);
        }

        m_driveTrain.setMotorArcadeDrive(throttle, turn);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
