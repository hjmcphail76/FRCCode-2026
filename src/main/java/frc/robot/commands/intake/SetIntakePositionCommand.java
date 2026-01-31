package frc.robot.commands.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.RobotState.IntakePositions;
import frc.robot.RobotState;
import frc.robot.RobotConstants.IntakeContants;
import frc.robot.utils.CowboyUtils;

public class SetIntakePositionCommand extends Command {
    IntakeSubsystem intakeSubsystem;
    IntakePositions position;

    public SetIntakePositionCommand(IntakeSubsystem intakeSubsystem, IntakePositions position) {
        intakeSubsystem.setIntakeSpeed(0);
        this.intakeSubsystem = intakeSubsystem;
        this.position = position;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        if (CowboyUtils.isSim()) {
            RobotState.intakePosition = position;
        } else {
            if (position == IntakePositions.RETRACTED) {
                if (intakeSubsystem.getDeploymentMotorEncoderRevs() < IntakeContants.ENCODER_REVOLUTIONS_TO_DEPLOY) {
                    intakeSubsystem.setDeploymentMotorSpeed(.2);
                } else {
                    intakeSubsystem.setDeploymentMotorSpeed(0);
                    RobotState.intakePosition = IntakePositions.DEPLOYED;
                }
            } else if (position == IntakePositions.DEPLOYED) {
                if (intakeSubsystem.getDeploymentMotorEncoderRevs() > 0) {
                    intakeSubsystem.setDeploymentMotorSpeed(-.2);
                } else {
                    intakeSubsystem.setDeploymentMotorSpeed(0);
                    RobotState.intakePosition = IntakePositions.RETRACTED;
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setDeploymentMotorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return position != RobotState.intakePosition;
    }

}