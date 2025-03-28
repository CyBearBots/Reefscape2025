package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.ElevatorConstants;

import java.io.Console;
import java.util.ArrayList;

import javax.management.ConstructorParameters;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.ElevatorConstants;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ElevatorSubsystem extends SubsystemBase{
    private SparkMax leadMotor;
    private SparkMax followMotor;

    SparkMaxConfig followerConfig = new SparkMaxConfig();
    SparkMaxConfig resetConfig = new SparkMaxConfig();
    
    private SparkClosedLoopController pidController;
    private ProfiledPIDController profiledPID;
    private double targetPosition;

    public ElevatorSubsystem(int leadMotorId, int followMotorId){
        leadMotor = new SparkMax(leadMotorId, MotorType.kBrushless);
        followMotor = new SparkMax(followMotorId, MotorType.kBrushless);

        pidController = leadMotor.getClosedLoopController();
        profiledPID = new ProfiledPIDController(ElevatorConstants.kProfiledP, ElevatorConstants.kProfiledI, ElevatorConstants.kProfiledD, 
            new TrapezoidProfile.Constraints(ElevatorConstants.maxVelocity, ElevatorConstants.maxAcceleration)); //tune maxVelocity and maxAcceleration in ElevatorConstants

        followerConfig.follow(leadMotor, false);

        followMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        resetConfig.idleMode(IdleMode.kBrake);
        resetConfig.smartCurrentLimit(40);

        resetConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.05)
        .i(0.0)
        .d(0.0)
        .outputRange(-1, 1);

        configureMotors();

    }

    private void configureMotors(){
        leadMotor.configure(resetConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followMotor.configure(resetConfig, null, PersistMode.kPersistParameters);
    }
    
    public SparkMax getLeadMotor() {
        return leadMotor;
    }

    public SparkMax getFollowMotor(){
        return followMotor;
    }

    public double getElevatorPosition() {
        return getLeadMotor().getEncoder().getPosition();
    }

    //holds elevator at position with SparkClosedLoopController kPositionControlType
    public void setTargetPosition(double position) {
        targetPosition = position;
        pidController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0); //note: other teams use revolutions which may be more efficient
    }

    //this sets the speed of the motor based off how far it is from the goal state/position
    //Position = the goal height
    public void setProfiledPIDPosition(double currentPosition, double goalPosition) {
        // if (position >= 0) {
        //     leadMotor.set(profiledPID.calculate(getElevatorPosition(), position)); //goes to the desired position with a calculated speed
        // } else {
        //     leadMotor.set(0);
        // }

        // if (currentPosition < goalPosition) {
        //     leadMotor.set(profiledPID.calculate(getElevatorPosition(), goalPosition));
        // } else if (currentPosition > goalPosition) {
        //     leadMotor.set(-profiledPID.calculate(getElevatorPosition(), goalPosition));
        // } else {
        //     leadMotor.set(0);
        // }
        currentPosition = getElevatorPosition();
        leadMotor.set(-profiledPID.calculate(currentPosition, goalPosition));
    }

    // public void setConfig(IdleMode idleMode) {
    //     for (SparkMax motor : sparkMaxMotors) {
    //         SparkMaxConfig config_ = new SparkMaxConfig();
    //         config_.idleMode(idleMode);
    
    //         motor.configure(config_, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    //     }
    // }

    // public void periodic() {
    //    // SmartDashboard.putNumber("Elevator encoder value", getEncoderMeters());
    // }
}
