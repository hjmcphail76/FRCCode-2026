package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.RobotState;



public interface IndexerSubsystemIO {

    @AutoLog
    public static class IndexerSubsystemIOInputs {
        public double conveyorMotorRMP = 0.0;
        public double conveyorMotorTempC = 0.0;

        public double indexerMotor = 0.0;
        public double indexerMotorTempC = 0.0;
        //TODO: add indexer state machine here for logging
    }

    default void updateInputs(IndexerSubsystemIOInputs inputs) {
    }

    default void setConveyorPercentSpeed(double percent){
    }

    default void setRollerPercentSpeed(double percent){
    }
}
