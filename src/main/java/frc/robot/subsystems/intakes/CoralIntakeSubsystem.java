package frc.robot.subsystems.intakes;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.revrobotics.AbsoluteEncoder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.Encoder;


public class CoralIntakeSubsystem extends SubsystemBase{

    private SparkMax motor;

    public CoralIntakeSubsystem(int motorID) {
        motor = new SparkMax(motorID, MotorType.kBrushless);
    }

    public void setSpeed(double speed){
        motor.set(speed);
    }

    //values of intake and eject should be opposite (signs)

    public void stopMotor(){
        motor.set(0);
        // can substitute with motor.stopMotor();
    }

    @Override
    public void periodic() {}

}



// //package frc.robot.Subsystems;

// ////import com.revrobotics.CANSparkMax;
// //import edu.wpi.first.wpilibj2.command.SubsystemBase;
// //import com.revrobotics.spark.SparkMax;
// //import com.revrobotics.spark.SparkLowLevel.MotorType;
// //import edu.wpi.first.wpilibj2.command.SubsystemBase;


// public class CoralIntakeSubsystem extends SubsystemBase{

//     private SparkMax intakeMotor;  //Initializes intake motor for coral

//     public CoralIntakeSubsystem (int centerMotorID){
//         intakeMotor = new SparkMax(centerMotorID, MotorType.kBrushless);
//     }

//     public void intakeCoral(double speed){
//         intakeMotor.set(speed);   //No idea what the intake speed will be so I am repacing it with the paramerter speed
//     }

//     public void intakeCoralFullStop(){
//         intakeMotor.set(0);  //Stops motor completeley
//     }


//     @Override
//     public void periodic() {
//     }
// }