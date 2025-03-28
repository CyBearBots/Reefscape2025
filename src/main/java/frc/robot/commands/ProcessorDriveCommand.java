// package frc.robot.commands;

// import java.sql.Driver;
// import java.util.*;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.IdealStartingState;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.Waypoint;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.VisionSubsystem;
// //import frc.robot.subsystems.LimelightSubsystem;
// import frc.robot.subsystems.swervedrive.SwerveSubsystem;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;

// public class ProcessorDriveCommand extends Command{
//     VisionSubsystem limelightSubsystem;
//     SwerveSubsystem swerveSubsystem;
//     double xPos;
//     double yPos;
//     double zPos;
//     List<Waypoint> points;
//     PathPlannerPath path;
//     int id;
//     Command pathCommand;
//     public double angle;
//     public int allianceFlip = -1;
//     public int offset = 0;

//     public ProcessorDriveCommand(VisionSubsystem limelightSubsystem, SwerveSubsystem swerveSubsystem){
//         this.limelightSubsystem = limelightSubsystem;
//         this.swerveSubsystem = swerveSubsystem;
//         addRequirements(limelightSubsystem);
//     }

//     @Override
//     public void initialize(){
//         if(DriverStation.getAlliance().get() == Alliance.Red){
//             id = 16;// processor april tag for red alliance 16
//             allianceFlip = -1;
//         } else if(DriverStation.getAlliance().get() == Alliance.Blue){
//             id = 3;// processor april tag for blue alliance 3
//             allianceFlip = 1;
//         }

//         if(limelightSubsystem.getId() == id){
//             xPos = limelightSubsystem.getAprilTagProperty(id, 0); // x
//             yPos = limelightSubsystem.getAprilTagProperty(id, 1); // y
//             zPos = limelightSubsystem.getAprilTagProperty(id, 2); // z
//             angle = limelightSubsystem.getAprilTagProperty(id, 3); // rotation angle
            
//             swerveSubsystem.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0.0)));

//             double bezierFlip = (xPos > 0) ? -90 : (xPos < 0) ? 90 : 0;

//             points = PathPlannerPath.waypointsFromPoses(
//                 new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
//                 new Pose2d(allianceFlip * (-1 * zPos + .25), 
//                         allianceFlip * (xPos - .04), 
//                         Rotation2d.fromDegrees(bezierFlip))
//             );

//             path = new PathPlannerPath(points, 
//                 new PathConstraints(1.0, 1.0, Math.PI/2, Math.PI/4),  // max velo, max accel, max turning velo, max turning accel
//                 new IdealStartingState(null, null),
//                 new GoalEndState(0.0, Rotation2d.fromDegrees((allianceFlip * -90) - angle))
//             );

//             pathCommand = AutoBuilder.followPath(path);
//             pathCommand.schedule();
//         } else {
//             this.cancel(); 
//         }
//     }

//     @Override
//     public void execute(){
//         if(id != -1){
//             if(pathCommand.isFinished()){
//                 this.cancel();
//             }
//         }
//     }

//     @Override
//     public void end(boolean interrupted){
//         swerveSubsystem.resetOdometry(new Pose2d(0,0,Rotation2d.fromDegrees(-90.0)));
//     }

//     @Override
//     public boolean isFinished(){
//         return false;
//     }
// }
