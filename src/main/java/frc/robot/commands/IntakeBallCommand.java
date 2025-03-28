package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakes.AlgaeIntakeSubsystem;
import frc.robot.Constants.AlgaeIntakeConstants;

public class IntakeBallCommand extends Command {
    private final AlgaeIntakeSubsystem algaeIntake;
    // private int algaeMotor1ID = AlgaeIntakeConstants.algaeLeftMotorID;
    // private int algaeMotor2ID = AlgaeIntakeConstants.algaeRightMotorID;
    private double speed;

    public IntakeBallCommand(AlgaeIntakeSubsystem algaeIntake, double speed) {
        this.algaeIntake = algaeIntake;
        this.speed = speed;
        addRequirements(algaeIntake);
    }
    
    public void initialize() {
        
    }

    public void execute() {
        algaeIntake.setSpeed(speed);
    }

    public void end(boolean interrupted) {
        algaeIntake.stopMotor();
    }

    public boolean isFinished() {
        return false;
    }

}
