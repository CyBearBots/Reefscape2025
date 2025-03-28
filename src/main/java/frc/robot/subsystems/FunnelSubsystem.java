package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class FunnelSubsystem extends SubsystemBase {

    private SparkMax motor;

    public FunnelSubsystem(int funnelMotorID) {
        motor = new SparkMax(funnelMotorID, MotorType.kBrushed);
    }

    public void setFunnelSpeed(double speed) {
        motor.set(speed);
    }

    public void stopMotor() {
        motor.set(0);
    }

}
