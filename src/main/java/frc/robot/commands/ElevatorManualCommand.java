package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorManualCommand extends Command{
    private ElevatorSubsystem e_subsystem;
    private double speed;

    private SlewRateLimiter speedLimiter = new SlewRateLimiter(ElevatorConstants.rateLimit);

    // DigitalInput topLimitSwitch = new DigitalInput(ElevatorConstants.topLimitSwitchChannel);
    // DigitalInput bottomLimitSwitch = new DigitalInput(ElevatorConstants.bottomLimitSwitchChannel);

    public ElevatorManualCommand(ElevatorSubsystem e_subsystem, double speed){
        this.e_subsystem = e_subsystem;
        this.speed = speed;

        addRequirements(e_subsystem);
    }

    // private void checkLimitSwitch() {
    //     if (!topLimitSwitch.get() || !bottomLimitSwitch.get()) {
    //         e_subsystem.getSparkMotor(liftMotor1Id).set(0);
    //         e_subsystem.getSparkMotor(liftMotor2Id).set(0);
    //     }
    // }

    @Override
    public void initialize(){
        try {
            
        } catch (Exception e) {
           e.printStackTrace();
        }
    }

    @Override
    public void execute(){
        // checkLimitSwitch();

        double limitedSpeed = speedLimiter.calculate(speed);

        SmartDashboard.putNumber("ElevatorManualSpeed", limitedSpeed);
        e_subsystem.getLeadMotor().set(limitedSpeed);

        SmartDashboard.putNumber("ElevatorManualPosition", e_subsystem.getLeadMotor().getEncoder().getPosition());
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        double position = e_subsystem.getLeadMotor().getEncoder().getPosition();
        e_subsystem.getLeadMotor().set(0);
        e_subsystem.setTargetPosition(position);
       // e_subsystem.setProfiledPIDPosition(position);
    }

}
