package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FunnelSubsystem;

public class FunnelCommand extends Command{
    private FunnelSubsystem f_subsystem;
    private double speed;
    private long startTime;
    private boolean automatic;

    public FunnelCommand(FunnelSubsystem f_subsystem, double speed /* , boolean automatic */) {
        this.f_subsystem = f_subsystem;
        this.speed = speed;
       // this.automatic = automatic;
        addRequirements(f_subsystem);
    }

    public void initialize() {
        System.out.println("Starting");
        startTime = System.currentTimeMillis();
    }

    public void execute() {
        f_subsystem.setFunnelSpeed(speed);
    }

    public boolean isFinished() {
        // if(automatic){
        //     return System.currentTimeMillis() - startTime >= 2000; // 1000 = 1s
        // }
        return false;
    }

    public void end(boolean interrupted) {
        f_subsystem.stopMotor();
    }
}
