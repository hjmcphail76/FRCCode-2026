package frc.robot.subsystems.indexer;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.RobotState;

public class IndexerSubsystemIOSim implements IndexerSubsystemIO {
    public IndexerSubsystemIOSim(){

    }

    @Override
    public void setConveyorPercentSpeed(double percent) {
    }

    @Override
    public void setIndexerPercentSpeed(double percent) {
    }

    @Override
    public void updateInputs(IndexerSubsystemIOInputs inputs) {
        inputs.conveyorMotorRMP = 0.0;
        inputs.conveyorMotorTempC = 0.0;

        inputs.indexerMotor = 0.0;
        inputs.indexerMotorTempC = 0.0;
    }
}
