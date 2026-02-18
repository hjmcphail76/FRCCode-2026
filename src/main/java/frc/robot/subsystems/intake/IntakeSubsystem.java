package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class IntakeSubsystem extends SubsystemBase {
    public IntakeSubsystemIO io;
    IntakeSubsystemIOInputsAutoLogged inputs = new IntakeSubsystemIOInputsAutoLogged();

    public IntakeSubsystem(IntakeSubsystemIO io) {
        this.io = io;
    }

    public void turnIntakeOn(){
        io.setPercentSpeed(.8);
    }

    public void turnIntakeOff(){
        io.setPercentSpeed(0);
    }

    public void setIntakeSpeed(double percent){
        io.setPercentSpeed(percent);
    }

    public void setDeploymentMotorSpeed(double percent){
        io.setDeploymentMotorPercentSpeed(percent);
    }

    public double getDeploymentMotorEncoderRevs(){
        return io.getDeploymentMotorEncoderRevs();
    }

    public Command startIntakingCommand(){
        return new SequentialCommandGroup(new InstantCommand(()->setIntakeSpeed(.5), this), new WaitCommand(.075), new InstantCommand(()->setIntakeSpeed(-75), this));
    }

    public Command stopIntakingCommand(){
        return new InstantCommand(()->setIntakeSpeed(0), this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("IntakeSubsystem", inputs);
    }
}