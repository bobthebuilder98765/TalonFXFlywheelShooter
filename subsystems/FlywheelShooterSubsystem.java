package frc.robot.subsystems;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FlywheelShooterSubsystem extends SubsystemBase{
    private final TalonFX motorPrimary  = new TalonFX(Constants.TalonFXConstants.TALONFX_MOTOR_ONE_ID);	
    private final TalonFX motorSecondary  = new TalonFX(Constants.TalonFXConstants.TALONFX_MOTOR_TWO_ID);
    private final PIDController PID = new PIDController(1, 1, 1);	//creates the PID controller with the kp,ki,kd values

    public FlywheelShooterSubsystem(){
        this.motorSecondary.setInverted(true);
    }

    public double getPrimaryMotorVelocityInRPM(){
        double velocityEngineOneInRPM = this.motorPrimary.getVelocity().getValueAsDouble() * 60;
        return velocityEngineOneInRPM;     
    }

    public double getSecondaryMotorVelocityInRPM(){
        double velocityEngineTwoInRPM = this.motorSecondary.getVelocity().getValueAsDouble() * 60;
        return velocityEngineTwoInRPM;
    }

    public double getAvgMotorsVelocityInRPM(){

       double velocityAvg = (getPrimaryMotorVelocityInRPM()+getSecondaryMotorVelocityInRPM())/2;
       return velocityAvg;
    }

    public double calculatePID(double setPoint){
        return this.PID.calculate(getAvgMotorsVelocityInRPM(),setPoint);
    }

    public void setMotorsSpeedInRPM(double speedInRPM){ //sets the inputted speed of RPM to motors
        this.motorPrimary.set(speedInRPM/60); //*** wrong function for setting
        this.motorSecondary.set(speedInRPM/60);
    }
}
                                                                           