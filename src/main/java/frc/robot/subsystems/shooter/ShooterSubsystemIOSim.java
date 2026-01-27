package frc.robot.subsystems.shooter;

public class ShooterSubsystemIOSim implements ShooterSubsystemIO {
    
    public void setPercentSpeed(double percent){
    }

    public void setRMP(double rpm){

    }

    @Override
    public void updateInputs(ShooterSubsystemIOInputs inputs) {
        inputs.motorRMP = 0;
        inputs.motorTempC = 0;
    }
}
