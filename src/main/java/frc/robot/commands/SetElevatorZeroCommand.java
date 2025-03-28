package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorZeroCommand extends Command{
    private ElevatorSubsystem eSubsystem;

    public SetElevatorZeroCommand(ElevatorSubsystem eSubsystem){
        this.eSubsystem = eSubsystem;
    }

    @Override
    public void initialize(){
        eSubsystem.getLeadMotor().getEncoder().setPosition(0);
        eSubsystem.getFollowMotor().getEncoder().setPosition(0);
    }

    @Override
    public void execute(){

    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
