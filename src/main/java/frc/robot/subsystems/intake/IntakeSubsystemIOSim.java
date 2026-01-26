package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.RobotState;

public class IntakeSubsystemIOSim implements IntakeSubsystemIO {
    public IntakeSubsystemIOSim(){

    }

    @Override
    public void setPercentSpeed(double percent){
        
    }

    @Override
    public void setRMP(double rpm){
        
    }

    @Override
    public void updateInputs(IntakeSubsystemIOInputs inputs) {
        inputs.motorRMP = 0;
        inputs.motorTempC = 0;
        inputs.deployMotorEncoderPosition = 0;
        inputs.intakePosition = RobotState.intakePosition;
    }
}
