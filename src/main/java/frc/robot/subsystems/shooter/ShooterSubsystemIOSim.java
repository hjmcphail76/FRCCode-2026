package frc.robot.subsystems.shooter;

public class ShooterSubsystemIOSim implements ShooterSubsystemIO {
    
    public void setPercentSpeed(double percent){
    }

    public void setRMP(double rpm){

    }
//Setting the motor rotations per minute & temp to 0 again?
    @Override
    public void updateInputs(ShooterSubsystemIOInputs inputs) {
        inputs.motorRMP = 0;
        inputs.motorTempC = 0;
    }
}
