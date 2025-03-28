package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralPivotConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SetArmPositionConstants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import com.revrobotics.spark.SparkMax;

public class SetArmPositionCommand extends Command {
    private ArmSubsystem a_subsystem;
    private double desiredPosition;

    SparkMax motor;
    RelativeEncoder relativeEncoder;

    double currentPosition;

    public SetArmPositionCommand(ArmSubsystem a_subsystem, double desiredPosition){
        this.a_subsystem = a_subsystem;
        this.desiredPosition = desiredPosition;

        motor = a_subsystem.getSparkMotor();
        relativeEncoder = motor.getEncoder();

        addRequirements(a_subsystem);
    }

    @Override
    public void initialize(){
        currentPosition = relativeEncoder.getPosition();
        SmartDashboard.putNumber("ArmPosition", currentPosition);
    }

    @Override
    public void execute(){
        if(currentPosition > desiredPosition){
            a_subsystem.setMotorSpeed(-CoralPivotConstants.speedDown);
        }else if(currentPosition < desiredPosition){
            a_subsystem.setMotorSpeed(CoralPivotConstants.speedUp);
        }
    }

    @Override
    public void end(boolean interrupted){
        a_subsystem.setMotorSpeed(0);

        currentPosition = relativeEncoder.getPosition();
        a_subsystem.setTargetPosition(currentPosition);
    }

    @Override
    public boolean isFinished(){
        currentPosition = relativeEncoder.getPosition();

        return Math.abs(currentPosition - desiredPosition) < SetArmPositionConstants.positionTolerance;
    }
}
