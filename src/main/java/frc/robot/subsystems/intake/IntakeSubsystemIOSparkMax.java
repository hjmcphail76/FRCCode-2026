package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IntakeSubsystemIOSparkMax implements IntakeSubsystemIO {
    SparkMax intakeMotor = new SparkMax(0, MotorType.kBrushless);
    SparkMax deploymentMotor = new SparkMax(0, MotorType.kBrushless);

    public IntakeSubsystemIOSparkMax(){

    }

    @Override
    public void updateInputs(IntakeSubsystemIOInputs inputs) {
        inputs.motorRMP = 0;
        inputs.motorTempC = 0;
        inputs.deployMotorEncoderPosition = 0;

    }
}
