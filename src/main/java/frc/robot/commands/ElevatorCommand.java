package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorCommand extends Command {
    private ElevatorSubsystem e_subsystem;
    //private int liftMotor1Id = ElevatorConstants.elevatorMotor1Id;
    //private int liftMotor2Id = ElevatorConstants.elevatorMotor2Id;
    private double targetHeight;

    private SlewRateLimiter speedLimiter = new SlewRateLimiter(ElevatorConstants.rateLimit);

    // DigitalInput topLimitSwitch = new DigitalInput(ElevatorConstants.topLimitSwitchChannel);
    // DigitalInput bottomLimitSwitch = new DigitalInput(ElevatorConstants.bottomLimitSwitchChannel);

    public ElevatorCommand(ElevatorSubsystem e_subsystem, double targetHeight) {
        this.e_subsystem = e_subsystem;
        this.targetHeight = targetHeight;
        addRequirements(e_subsystem);
    }

    // private void checkLimitSwitch() {
    //     if (!topLimitSwitch.get() || !bottomLimitSwitch.get()) {
    //         e_subsystem.getSparkMotor(liftMotor1Id).set(0);
    //         e_subsystem.getSparkMotor(liftMotor2Id).set(0);
    //     }
    // }

    @Override
    public void initialize() {
        try {
            //e_subsystem.setConfig(IdleMode.kCoast);
        } catch (Exception e) {
            e.printStackTrace();
        }
        // double position = e_subsystem.getLeadMotor().getEncoder().getPosition();
        // e_subsystem.setProfiledPIDPosition(position, targetHeight);
    }

    @Override
    public void execute() {
        //checkLimitSwitch();

        double currentPosition1 = e_subsystem.getLeadMotor().getEncoder().getPosition();
        //double currentPosition2 = e_subsystem.getFollowMotor().getEncoder().getPosition();

        SmartDashboard.putNumber("ElevatorPosition", currentPosition1);
        //SmartDashboard.putNumber("CurrentPosition2", currentPosition2);

        // double encoderPositionForHeight = targetHeight * ElevatorConstants.elevatorConversionFactor;
        double encoderPositionForHeight = targetHeight * ElevatorConstants.elevatorConversionFactor;


        // SmartDashboard.putNumber("ConversionFactor", ElevatorConstants.elevatorConversionFactor);
        SmartDashboard.putNumber("ElevatorTargetHeight", encoderPositionForHeight);

        if (encoderPositionForHeight > currentPosition1 /*&& topLimitSwitch.get()*/) { //elevator goes up
            // e_subsystem.getLeadMotor().set(speedLimiter.calculate(ElevatorConstants.liftMotor1SpeedUp));
            
            e_subsystem.getLeadMotor().set((ElevatorConstants.liftMotor1SpeedUp));
            
            // e_subsystem.setProfiledPIDPosition(currentPosition1, encoderPositionForHeight);
        } else if (encoderPositionForHeight < currentPosition1 /* && bottomLimitSwitch.get()*/) { //eleavtor goes down
            // e_subsystem.getLeadMotor().set(speedLimiter.calculate(-ElevatorConstants.liftMotor1SpeedDown));
            
            e_subsystem.getLeadMotor().set((-ElevatorConstants.liftMotor1SpeedDown));

            // e_subsystem.setProfiledPIDPosition(currentPosition1, encoderPositionForHeight);
        } else {
            e_subsystem.getLeadMotor().set(0);
        }

    }

    @Override
    public void end(boolean interrupted) {
        double position = e_subsystem.getLeadMotor().getEncoder().getPosition();
        e_subsystem.getLeadMotor().set(0);
        e_subsystem.setTargetPosition(position);
        SmartDashboard.putNumber("ElevatorHoldHeight", position);
    }

    @Override
    public boolean isFinished() {
        double currentPosition1 = e_subsystem.getLeadMotor().getEncoder().getPosition();

        double encoderPositionForHeight = targetHeight * ElevatorConstants.elevatorConversionFactor;

        return Math.abs(currentPosition1 - encoderPositionForHeight) < ElevatorConstants.elevatorTolerance;
    }
}
