package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakes.CoralIntakeSubsystem;
// import frc.robot.Constants.CoralIntakeConstants;

public class CoralIntakeCommand extends Command{
    private final CoralIntakeSubsystem coralIntake;
    private double speed;
    private boolean automatic;
    private long startTime;

    public CoralIntakeCommand(CoralIntakeSubsystem coralIntake, double speed /*, boolean automatic */) {
        this.coralIntake = coralIntake;
        this.speed = speed;
        // this.automatic = automatic;
        addRequirements(coralIntake);
    }
    
    public void initialize() {
        System.out.println("Start Coral Intake commnand!"); //for debugging purposes
        startTime = System.currentTimeMillis();
    }

    public void execute() {
        coralIntake.setSpeed(speed);
    }

    public void end(boolean interrupted) {
        System.out.println("Stopping Coral Intake command!"); //for debugging purposes
        coralIntake.stopMotor();
    }

    public boolean isFinished() {
        // if(automatic){
        //     return System.currentTimeMillis() - startTime >= 1000; // 1000 = 1s
        // }
        return false;
    }
}
