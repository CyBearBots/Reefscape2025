// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

import edu.wpi.first.math.Matrix;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class ElevatorConstants {
    public static final double[] L0 = {Units.feetToMeters(0), -18.15}; // Elevator base level, Arm flat out

    public static final double[] L1 = {(Units.feetToMeters(3.927083)) - 0.25, -30.0}; // Elevator trough level
    public static final double[] L2 = {(Units.feetToMeters(2.65625)) - 0.61, -4.8}; // Elevator first bar level, Arm angled up
    public static final double[] L3 = {(Units.feetToMeters(3.927083)) + 0.05}; // Elevator second bar level
    public static final double[] L4 = {(Units.feetToMeters(6)) + 0.03, -9.0, -20.15}; // Elevator third bar level, Arm angled up

    public static final double[] LP = {0.28, -31.6}; // Elevator processor level, Arm angled down
    public static final double[] LB = {(Units.feetToMeters(6)) + 0.05, -12.5}; // Elevator barge level, Arm angled up

    public static final double[] LF = {0.78, 35.0};

    public static final double rateLimit = 1.0;

    public static final double elevatorTolerance = 0.1;

    public static final double liftMotor1SpeedUp = 0.6;
    public static final double liftMotor1SpeedDown = 0.15;
    
    public static final double encoderTicksPerRevolution = 42; // Spark Neo 
    // public static final double elevatorPulleyCircumference = 0.07853; // Diameter of pulley wheel times pi (in meters)
    // public static final double elevatorConversionFactor = elevatorPulleyCircumference / encoderTicksPerRevolution; // Convert distance to encoder ticks
    public static final double elevatorConversionFactor = 25.9013388; // 25.9013388

    public static final int topLimitSwitchChannel = 13;
    public static final int bottomLimitSwitchChannel = 14;

    public static final int elevatorMotor1Id = 11; // Left
    public static final int elevatorMotor2Id = 12;

    // These are the PID values for the SparkClosedLoopControlller (controls position) and ProfiledPIDController (controls speed and acceleration), MUST tune these values
    public static final double kControllerP = 0.05;
    public static final double kControllerI = 0;
    public static final double kControllerD = 0;

    //The following constants are for the ProfiledPIDController
    public static final double kProfiledP = 3.0; //tune   5
    public static final double kProfiledI = 0;
    public static final double kProfiledD = 0.15; //tune   0.2
    
    public static final double maxVelocity = 0.5;
    public static final double maxAcceleration = 0.20;
  }

  public static class SetArmPositionConstants{
    public static final double positionTolerance = 0.1;
  }

  public static class CoralPivotConstants{
    public static final int coralPivotMotor1Id = 13;
    public static final double speedUp = 0.40;
    public static final double speedDown = 0.1;
  }

  public static class CoralIntakeConstants {
    public static final int coralIntakeMotorId = 14;
    public static final double intakeSpeed = -0.2;
    public static final double ejectSpeed = 0.5; 
  }

  public static class AlgaeIntakeConstants {
    public static final int algaeLeftMotorId = 15;
    public static final double intakeSpeed = -0.4; 
    public static final double ejectSpeed = 1; 
  }

  public static class FunnelConstants {
    public static final int funnelMotorId = 17; 
    public static final double funnelSpeed = -0.35; 
  }
  
  public static class TagConstants {
    public static final double X_REEF_ALIGNMENT_P = 2.9; // Tune value 
    public static final double Y_REEF_ALIGNMENT_P = 3; // Tune value 
    public static final double ROT_REEF_ALIGNMENT_P = 0.03; // Tune value


    /*
    Tolerance: how much error you allow (smaller = more accurate). Too large will make it stop and not be aligned
    Setpoints: put your robot on the field where you want it to be at the end of alignment (where it is when is scores)
    
    Open limelight web --> advanced tab --> read robot position from there (to get the setpoint value)
    Set the view to "Robot Pose in Target Space"
    X Setpoint = TX       Y Setpoint = TZ         rot setpoint = RY
    Since camera isn't sensored w/ intakes, 2 different values (for y-setpoint) are needed for the left and right pipes 
    */
    public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0; //Rotation -- tune value
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1; 

    public static final double X_SETPOINT_REEF_ALIGNMENT = -15.92; //Vertical Pose -- tune value   -0.3    -15.92
    public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.02; 

    //because our limelight isn't centered with the arm, we need 2 different Y_SETPOINTS --> 1 for the Left Pipe and 1 for the Right Pipe
    public static final double Y_SETPOINT_REEF_ALIGNMENT = 9.97; //Horizontal Pose -- tune value        0.26
    // public static final double Y_SETPOINT_LEFT_REEF_ALIGNMENT = 0;  //find value through Limelight web (TZ)
    // public static final double Y_SETPOINT_RIGHT_REEF_ALIGNMENT = 0;
    public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02; 

    public static final double DONT_SEE_TAG_WAIT_TIME = 1; //how long we don't see the tag before command ends
    public static final double POSE_VALIDATION_TIME = 0.3; /*Tune if needed - this is the amount of time needed to be aligned before command finishes */
  }


  // public static class cageConstants {

  // }

  public static class DriveConstants {
    public static final int intakeID = 20;
    public static final int leftShooterID = 21;
    public static final int rightShooterID = 22;
    public static final int raiseID = 31;
    public static final int leftLiftID = 23;
    public static final int rightLiftID = 24;

    public static final int kDriverControllerPort1 = 0;
    public static final int kDriverControllerPort2 = 1;
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;

    public static final double LEFT_X_DEADBAND = 0.1;
    
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

   public static final class VisionConstants{
    public static final  double maximumAmbiguity                = 0.25;
    public static final double debouncetimeMilli = 15;

    // public static final class CameraTemplate{
    //   public static final String name = "tempalte"; // tempalte?
    //   public static  Rotation3d robotToCamTransform = new Rotation3d(0, Units.degreesToRadians(18), 0);
    //   public static  Translation3d robotToCamTranslation=new Translation3d(Units.inchesToMeters(-4.628),
    //   Units.inchesToMeters(-10.687),
    //   Units.inchesToMeters(16.129));
    //   public static  Matrix<N3, N1> singleTagStdDevs= VecBuilder.fill(2, 2, 8);
    //   public static  Matrix<N3, N1> multiTagStdDevsMatrix= VecBuilder.fill(0.5, 0.5, 1); 
    // }

    public static final class ReefCamera{
      public static final String name = "Limelight7504";
      public static  Rotation3d robotToCamTransform = new Rotation3d(0, 0, Units.degreesToRadians(0)); //-45
      public static  Translation3d robotToCamTranslation=new Translation3d(Units.inchesToMeters(0),
      Units.inchesToMeters(0),
      Units.inchesToMeters(0));
      public static  Matrix<N3, N1> singleTagStdDevs= VecBuilder.fill(4, 4, 8);
      public static  Matrix<N3, N1> multiTagStdDevsMatrix= VecBuilder.fill(0.5, 0.5, 1); 
      public static InterpolatingDoubleTreeMap stdDevsMap = new InterpolatingDoubleTreeMap();
    }

    // public static final class ChuteCamera{//used only for streaming
    //   public static final String name = "Arducam_B0495_(USB3_2.3MP)";
    // }

    public static final double leftAlignmentX = .2435737274077523; //meters
    public static final double leftAlignmentY = 0.26;
    public static final double rightAlignmentX = .2435737274077523;
    public static final double rightAlignmentY = 0.6354460866293814;
    public static final double thetaAlignment = -Math.PI/2; //degrees
    public static double maxAlignmentDistance = 1.5;

    public static final double xTolerance = .05;
    public static final double yTolerance = .05;
    public static final double thetaTolerance = .05;
  }

  public static class SwerveConstants {
    public static final double kPX = .01; // 1.5
    public static final double kPY = .01; // 1.25
    public static final double kPTheta = .01; // 1.5
    public static final double kXTolerance = 0.01; //Meters
    public static final double kYTolerance = 0.01; //Meters
    public static final double kThetaTolerance = 0.01;
    public static double kStoredRadius = 3.9527559/2; // to be configured later
    public static double kDrivebaseRadius = .409;
  }

  public static class FieldPositions {
    //Blue
    public static final Pose2d L17 = new Pose2d(4.019, 2.913, new Rotation2d(Math.toRadians(150)));
    public static final Pose2d L18 = new Pose2d(3.357, 3.829, new Rotation2d(Math.toRadians(90)));
    public static final Pose2d L19 = new Pose2d(3.75, 5.100, new Rotation2d(Math.toRadians(30)));
    public static final Pose2d L20 = new Pose2d(5.142, 5.263, new Rotation2d(Math.toRadians(-30)));
    public static final Pose2d L21 = new Pose2d(5.669, 3.664, new Rotation2d(Math.toRadians(-90)));
    public static final Pose2d L22 = new Pose2d(5.142, 3.148, new Rotation2d(Math.toRadians(210)));

    public static final Pose2d R17 = new Pose2d(4.328, 2.764, new Rotation2d(Math.toRadians(150)));
    public static final Pose2d R18 = new Pose2d(3.304, 3.510, new Rotation2d(Math.toRadians(90)));
    public static final Pose2d R19 = new Pose2d(3.439, 4.923, new Rotation2d(Math.toRadians(30)));
    public static final Pose2d R20 = new Pose2d(4.657, 5.436, new Rotation2d(Math.toRadians(-30)));
    public static final Pose2d R21 = new Pose2d(5.679, 4.479, new Rotation2d(Math.toRadians(-90)));
    public static final Pose2d R22 = new Pose2d(5.483, 3.261, new Rotation2d(Math.toRadians(210)));

    //Red
    public static final Pose2d L6 = new Pose2d(13.65, 2.92, new Rotation2d(Math.toRadians(210)));
    public static final Pose2d L7 = new Pose2d(14.32, 4.00, new Rotation2d(Math.toRadians(-90)));
    public static final Pose2d L8 = new Pose2d(13.72, 5.12, new Rotation2d(Math.toRadians(-30)));
    public static final Pose2d L9 = new Pose2d(12.45, 5.16, new Rotation2d(Math.toRadians(30)));
    public static final Pose2d L10 = new Pose2d(11.78, 4.08, new Rotation2d(Math.toRadians(90)));
    public static final Pose2d L11 = new Pose2d(12.38, 2.96, new Rotation2d(Math.toRadians(150)));

    public static final Pose2d R6 = new Pose2d(14.14, 3.20, new Rotation2d(Math.toRadians(210)));
    public static final Pose2d R7 = new Pose2d(14.32, 4.57, new Rotation2d(Math.toRadians(-90)));
    public static final Pose2d R8 = new Pose2d(13.22, 5.40, new Rotation2d(Math.toRadians(-30)));
    public static final Pose2d R9 = new Pose2d(11.95, 4.87, new Rotation2d(Math.toRadians(30)));
    public static final Pose2d R10 = new Pose2d(11.78, 3.51, new Rotation2d(Math.toRadians(90)));
    public static final Pose2d R11 = new Pose2d(12.87, 2.67, new Rotation2d(Math.toRadians(150)));
    public static final Pose2d BARGE_START = new Pose2d(7.449, 6.134, new Rotation2d(Math.toRadians(-30)));
    public static final Pose2d PROCESSOR_START = new Pose2d(7.449, 6.134, new Rotation2d(Math.toRadians(-30)));
    public static final List<Pose2d> kLeftReefPoses=Arrays.asList(
    L17, L18, L19, L20, L21, L22, L6, L7, L8, L9, L10, L11
    );
    public static final List<Pose2d> kRightReefPoses = Arrays.asList(
      R17, R18, R19, R20, R21, R22, R6, R7, R8, R9, R10, R11
    );

    public static final Translation2d BLUE_REEF_CENTER = new Translation2d(4.5, 4);
    public static final Translation2d RED_REEF_CENTER = new Translation2d(13, 4);


    public static final int [] kReefIDs = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
    public static boolean isReefID(int id) {
      for(int i: kReefIDs) {
        if(id==i) return true;
      }
      return false;
    }

    public static final Pose2d BLUE_LEFT_CORAL_STATION_PICKUP = new Pose2d(new Translation2d(1.2,7), Rotation2d.fromDegrees(120));
    public static final Pose2d BLUE_CLIMB_AREA = new Pose2d(new Translation2d(7.638,6.174), Rotation2d.fromDegrees(0));
    public static final Pose2d BLUE_PROCESSOR = new Pose2d(new Translation2d(6.332,.52), Rotation2d.fromDegrees(-90));
    public static class StartPositions {
      public static final Pose2d ODO_TEST = new PathPlannerAuto("Odom Testing").getStartingPose();
    }
  }

}
