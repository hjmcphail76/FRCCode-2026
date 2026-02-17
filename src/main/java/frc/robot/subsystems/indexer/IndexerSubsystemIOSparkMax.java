package frc.robot.subsystems.indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import frc.robot.RobotConstants.PortConstants;
import frc.robot.RobotState.IntakePositions;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IndexerSubsystemIOSparkMax implements IndexerSubsystemIO {
    SparkMax conveyorMotor;
    SparkMax indexerMotor;

    SparkMaxConfig conveyerMotorConfig;
    SparkMaxConfig indexerMotorConfig;

    public IndexerSubsystemIOSparkMax() {
        conveyorMotor = new SparkMax(PortConstants.CAN.CONVEYOR_MOTOR, MotorType.kBrushless);
        indexerMotor = new SparkMax(PortConstants.CAN.INDEXER_MOTOR, MotorType.kBrushless);

        conveyerMotorConfig = new SparkMaxConfig();
        conveyerMotorConfig.smartCurrentLimit(30).idleMode(IdleMode.kCoast);
        conveyorMotor.configure(conveyerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        indexerMotorConfig = new SparkMaxConfig();
        indexerMotorConfig.smartCurrentLimit(30).idleMode(IdleMode.kCoast);
        indexerMotor.configure(conveyerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setConveyorPercentSpeed(double percent) {
        conveyorMotor.set(percent);
    }

    @Override
    public void setIndexerPercentSpeed(double percent) {
        indexerMotor.set(percent);
    }

    @Override
    public void updateInputs(IndexerSubsystemIOInputs inputs) {
        inputs.conveyorMotorRMP = conveyorMotor.getEncoder().getVelocity();
        inputs.conveyorMotorTempC = conveyorMotor.getMotorTemperature();

        inputs.indexerMotor = indexerMotor.getEncoder().getVelocity();
        inputs.indexerMotorTempC = indexerMotor.getMotorTemperature();

    }
}
