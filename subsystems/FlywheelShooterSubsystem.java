package frc.robot.subsystems;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Subsystem.Talonfx.ID;
import frc.robot.Constants.Subsystem.Talonfx.PIDValues;



public class FlywheelShooterSubsystem extends SubsystemBase{
    private final TalonFX motorPrimary  = new TalonFX(ID.TALONFX_MOTOR_PRIMARY_ID);	
    private final TalonFX motorSecondary  = new TalonFX(ID.TALONFX_MOTOR_SECONDARY_ID);
    final VelocityVoltage VelocityPIDResultRequest = new VelocityVoltage(0).withSlot(0);


    public FlywheelShooterSubsystem(){
        configureMotor(motorPrimary);
        motorSecondary.setControl(new Follower(ID.TALONFX_MOTOR_PRIMARY_ID, true)); //sets the secondary motor to follow the main one but reversed 
    }

    private void configureMotor(TalonFX motor) {
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = PIDValues.KP;
        slot0Configs.kI = PIDValues.KI;
        slot0Configs.kD = PIDValues.KD;
        motor.getConfigurator().apply(slot0Configs);
    }



    public double getPrimaryMotorVelocityInRPM(){
        return this.motorPrimary.getVelocity().getValueAsDouble() * 60;
    }

    public double getSecondaryMotorVelocityInRPM(){
        return this.motorSecondary.getVelocity().getValueAsDouble() * 60;
    }

    public double getAvgMotorsVelocityInRPM(){

       return (getPrimaryMotorVelocityInRPM()+getSecondaryMotorVelocityInRPM())/2;
    }
    public void setPIDSetPointInRPS(double setPoint){
        motorPrimary.setControl(this.VelocityPIDResultRequest.withVelocity(setPoint));  
    }


    public void stopMotors(){
        this.motorPrimary.stopMotor();
        this.motorSecondary.stopMotor();

    }

}
                                                                           