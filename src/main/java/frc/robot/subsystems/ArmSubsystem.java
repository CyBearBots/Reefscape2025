package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import java.util.ArrayList;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;


import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.spark.SparkBase.*;


public class ArmSubsystem extends SubsystemBase{
    private SparkMax motor;

    // private final PIDController pidController;
    private SparkClosedLoopController pidController;

    private double targetPosition;

    SparkMaxConfig resetConfig = new SparkMaxConfig();

    public ArmSubsystem(int motorId){
        motor = new SparkMax(motorId, MotorType.kBrushless);

        resetConfig.idleMode(IdleMode.kBrake);
        resetConfig.smartCurrentLimit(30, 30);

        pidController = motor.getClosedLoopController();

        /*
        pidController = new PIDController(0.1, 0, 0);
       
        resetConfig.closedLoop.p(0.1);
        resetConfig.closedLoop.i(0.0);
        resetConfig.closedLoop.p(0.0);
         */
        

        resetConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0.0)
        .d(0.0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        motor.configure(resetConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    public void setMotorSpeed(double speed){
        motor.set(speed);
    }

    public void stopMotor(){
        motor.set(0);
    }

    public SparkMax getSparkMotor() {
        return motor;
    }

    public void setTargetPosition(double position) {
        targetPosition = position;
        pidController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    // @Override
    // public void periodic() {
    //     double currentPosition = motor.getAbsoluteEncoder().getPosition();
    //     double pidOutput = pidController.calculate(currentPosition, targetPosition);
    //     motor.set(pidOutput);

    //     SmartDashboard.putNumber("Arm Position", currentPosition);
    //     SmartDashboard.putNumber("Target Position", targetPosition);
    // }
}
