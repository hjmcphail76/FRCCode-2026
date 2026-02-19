package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.RobotState;



public interface FeederSubsystemIO {

    @AutoLog
    public static class FeederSubsystemIOInputs {
        public double feederMotorRPM = 0.0;
        public double feederMotorTempC = 0.0;

        //TODO: add feeder state machine here for logging
    }

    default void updateInputs(FeederSubsystemIOInputs inputs) {
    }

    default void setFeederPercentSpeed(double percent){
    }
}
