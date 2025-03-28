package frc.robot.subsystems.intakes;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.AbsoluteEncoder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntakeSubsystem extends SubsystemBase{

    private SparkMax leftMotor;
    //private SparkMax rightMotor;

    public AlgaeIntakeSubsystem(int leftMotorID) {
        leftMotor = new SparkMax(leftMotorID, MotorType.kBrushless);
        //rightMotor = new SparkMax(rightMotorID, MotorType.kBrushless);
    }

    public void setSpeed(double speed) {
        leftMotor.set(speed);
        //rightMotor.set(speed);
    }

    //values of speed for intake and eject should be opposite

    public void stopMotor(){
        leftMotor.stopMotor();
        //rightMotor.stopMotor();
    }
    
    @Override
    public void periodic() {}
}