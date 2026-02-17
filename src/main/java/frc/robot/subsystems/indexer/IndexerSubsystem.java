package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    public IndexerSubsystemIO io;
    IndexerSubsystemIOInputsAutoLogged inputs = new IndexerSubsystemIOInputsAutoLogged();

    public IndexerSubsystem(IndexerSubsystemIO io) {
        this.io = io;
    }

    public Command startIndexing(){
        return new InstantCommand(()->{
            io.setConveyorPercentSpeed(.75);
            io.setIndexerPercentSpeed(.5);
        }, this);
    }
    public Command stopIndexing(){
        return new InstantCommand(()->{
            io.setConveyorPercentSpeed(0);
            io.setIndexerPercentSpeed(0);
        }, this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("IndexerSubsystem", inputs);
    }
}