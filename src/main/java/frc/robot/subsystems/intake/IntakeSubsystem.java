package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterSubsystemIO.ShooterSubsystemIOInputs;

public class IntakeSubsystem extends SubsystemBase {
    public IntakeSubsystemIO io;
    IntakeSubsystemIOInputsAutoLogged inputs = new IntakeSubsystemIOInputsAutoLogged();

    public IntakeSubsystem(IntakeSubsystemIO io) {
        this.io = io;
    }

    public void turnIntakeOn(){
        io.setPercentSpeed(.5);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("IntakeSubsystem", inputs);
    }
}