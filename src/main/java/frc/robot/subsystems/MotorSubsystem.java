// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Dictionary;
import java.util.Hashtable;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorSubsystem extends SubsystemBase {
    

    ArrayList<VictorSPX> victorMotors = new ArrayList<>();
    public ArrayList<SparkMax> sparkMaxMotors = new ArrayList<>();

    /** Creates a new ExampleSubsystem. */
    public MotorSubsystem() {


    }

    public MotorSubsystem(int[] victorID){
        if(victorID.length > 1){
            for(int i = 0; i < victorID.length; i++){
                victorMotors.add(new VictorSPX(victorID[i]));
            }
        }

    }

    public MotorSubsystem(int[] victorID, int[] sparkBrushedID, int[] sparkBrushlessID){
        if(victorID.length > 1){
            for(int i = 0; i < victorID.length; i++){
                victorMotors.add(new VictorSPX(victorID[i]));
            }
        }

        if(sparkBrushedID.length > 1){
            for(int i = 0; i < sparkBrushedID.length; i++){
                sparkMaxMotors.add(new SparkMax(sparkBrushedID[i], MotorType.kBrushed));
            }
        }

        if(sparkBrushlessID.length > 1){
            for(int i = 0; i < sparkBrushlessID.length; i++){
                sparkMaxMotors.add(new SparkMax(sparkBrushlessID[i], MotorType.kBrushless));
            }
        }

    }




   public void addVictorMotor(int id){
        victorMotors.add(new VictorSPX(id));
   }


   public void brakeVictorMotor(int id, double percent){
        victorMotors.get(id).setNeutralMode(NeutralMode.Brake);

    
   }
   public VictorSPX getVictorMotor(int id) {
    for(int i = 0; i < victorMotors.size(); i++){
        if(victorMotors.get(i).getDeviceID() == id){
            return victorMotors.get(i);
        }
    }
    return null;
   }

   public void addSparkMotor(int id, MotorType motorType){
    sparkMaxMotors.add(new SparkMax(id, motorType));
}

public SparkMax getSparkMotor(int id) {
for(int i = 0; i < sparkMaxMotors.size(); i++){
    if(sparkMaxMotors.get(i).getDeviceId() == id){
        return sparkMaxMotors.get(i);
    }
}
 return null;
}


    
  /**
   * Example command factory method.
   *
   * @return a command
   */


  @Override
  public void periodic() {
    
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
