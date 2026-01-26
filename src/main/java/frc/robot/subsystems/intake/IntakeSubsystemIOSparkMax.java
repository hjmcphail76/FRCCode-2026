package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import frc.robot.RobotState.IntakePositions;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeSubsystemIOSparkMax implements IntakeSubsystemIO {
    SparkMax intakeMotor;
    SparkMax deploymentMotor;

    SparkMaxConfig intakeMotorConfig;
    SparkMaxConfig deploymentMotorConfig;

    public IntakeSubsystemIOSparkMax(){
    intakeMotor = new SparkMax(0, MotorType.kBrushless);
    deploymentMotor = new SparkMax(0, MotorType.kBrushless);

    SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    intakeMotorConfig.smartCurrentLimit(30).idleMode(IdleMode.kBrake);
    intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
}

    @Override
    public void setPercentSpeed(double percent){
        intakeMotor.set(percent);
    }

    @Override
    public void setRMP(double rpm){
        
    }

    @Override
    public void setIntakePosition(IntakePositions state){

    }

    @Override
    public void updateInputs(IntakeSubsystemIOInputs inputs) {
        inputs.motorRMP = 0;
        inputs.motorTempC = 0;
        inputs.deployMotorEncoderPosition = 0;

    }
}
