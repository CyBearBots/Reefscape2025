package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SetArmZeroCommand extends Command{
    private ArmSubsystem aSubsystem;

    public SetArmZeroCommand(ArmSubsystem aSubsystem){
        this.aSubsystem = aSubsystem;
    }

    @Override
    public void initialize(){
        aSubsystem.getSparkMotor().getEncoder().setPosition(0);
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
