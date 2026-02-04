package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterSubsystemIO.ShooterSubsystemIOInputs;

public class ShooterSubsystem extends SubsystemBase {
    public ShooterSubsystemIO io;
    ShooterSubsystemIOInputsAutoLogged inputs = new ShooterSubsystemIOInputsAutoLogged();

    public ShooterSubsystem(ShooterSubsystemIO io) {
        this.io = io;
    }
//sets shooter rotation speed
    public void setPercentSpeed(double percent){
        io.setPercentSpeed(percent);
    }

    public void setRMP(double rpm){
        io.setRMP(rpm);
    }
//Updates the inputs after setting the speed
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ShooterSubsystem", inputs);

        //Logger.recordOutput(getName(), null);
    }
}