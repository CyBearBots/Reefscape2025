package frc.robot.commands;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class CoralPivotCommand extends Command{
    private final ArmSubsystem a_subsystem;
    private final double speed;

    SparkMax motor;
    //SparkAbsoluteEncoder absoluteEncoder;
    RelativeEncoder absoluteEncoder;

    double currentPosition;

    public CoralPivotCommand(ArmSubsystem a_subsystem, double speed){
        this.a_subsystem = a_subsystem;
        this.speed = speed;

        motor = a_subsystem.getSparkMotor();
        //absoluteEncoder = motor.getAbsoluteEncoder();
        absoluteEncoder = motor.getEncoder();

        addRequirements(a_subsystem);
    }

    @Override
    public void initialize() {
        currentPosition = absoluteEncoder.getPosition(); 
        SmartDashboard.putNumber("CoralPivotPosition", currentPosition);
    }

    @Override
    public void execute(){
        currentPosition = absoluteEncoder.getPosition();
        a_subsystem.setMotorSpeed(speed);
    }    

    @Override
    public void end(boolean interrupted) {
        a_subsystem.setTargetPosition(currentPosition);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
