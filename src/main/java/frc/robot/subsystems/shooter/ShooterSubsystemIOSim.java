package frc.robot.subsystems.shooter;

import frc.robot.RobotState.ShooterStates;

public class ShooterSubsystemIOSim implements ShooterSubsystemIO {
    
    public void setPercentSpeed(double percent){
    }

    public void setRMP(double rpm){

    }

    @Override
    public void updateInputs(ShooterSubsystemIOInputs inputs) {
        inputs.motorRMP = 0;
        inputs.motorTempC = 0;
        inputs.shooterState = ShooterStates.OFF;
    }
}
