package frc.robot.subsystems.feeder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.RobotState;

public class FeederSubsystemIOSim implements FeederSubsystemIO {
    public FeederSubsystemIOSim(){

    }

    @Override
    public void setFeederPercentSpeed(double percent) {
    }


    @Override
    public void updateInputs(FeederSubsystemIOInputs inputs) {
        inputs.feederMotorRPM = 0.0;
        inputs.feederMotorTempC = 0.0;
    }
}
