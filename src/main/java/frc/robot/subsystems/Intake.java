
package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Constants;

public class Intake extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private double kFF;
  private double kP;
  private double kI;
  private double kD;

  private double kI_Zone;
  private double allowableError;
  private double maxVel;
  private double maxAccel;
  private double gearRatio = 1.0/3.0;

  private CANSparkMax intakeMotor[] ={
        new CANSparkMax(Constants.intakeMotor, MotorType.kBrushless),
  };
  private CANPIDController canPidController = intakeMotor[0].getPIDController();

  DoubleSolenoid intakePiston = new DoubleSolenoid(Constants.pcmOne, Constants.intakePistonForward, Constants.intakePistonReverse);

  public Intake() {
    for(CANSparkMax intakeMotor: intakeMotor) {
      intakeMotor.restoreFactoryDefaults();
      intakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }
    canPidController.setFF(kFF);
    canPidController.setP(kP);
    canPidController.setI(kI);
    canPidController.setIZone(kI_Zone);
    canPidController.setD(kD);
    canPidController.setSmartMotionMaxVelocity(maxVel, 9);
    canPidController.setSmartMotionMaxVelocity(maxAccel, 0);
    canPidController.setSmartMotionAllowedClosedLoopError(allowableError, 0);
    intakeMotor[0].setInverted(false);
  }

  public boolean getintakePistonExtendStatus(){
    return intakePiston.get() == DoubleSolenoid.Value.kForward ? true : false;
  }

  public void setintakePiston(boolean state){
    intakePiston.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }

  public void setIntakePercentOutput(double value){
    intakeMotor[0].set(value);
  }

  public void setIntakeRPM(double rpm){
    double setpoint =  rpm * gearRatio;
    canPidController.setReference(setpoint, ControlType.kSmartVelocity);
  }

  @Override
  public void periodic() {
  }
}
