// package frc.robot.commands;


// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.swervedrive.SwerveSubsystem;
// import frc.robot.subsystems.VisionSubsystem;
// import frc.robot.Constants.TagConstants;
// import java.util.Optional;


// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonTrackedTarget;


// import edu.wpi.first.math.geometry.Pose3d;


// public class AlignToReef extends Command {
//     private final SwerveSubsystem swerve;
//     private final VisionSubsystem visionSubsystem;
//     private final PhotonCamera camera = new PhotonCamera("Limelight7504");
//     private PIDController xController, yController, rotController;
//     private boolean isRightScore; // true means right pipe, false means left pipe (on reef)
//     private Timer dontSeeTagTimer, stopTimer;


//     public AlignToReef(boolean isRightScore, SwerveSubsystem swerve, VisionSubsystem visionSubsystem) {
//         xController = new PIDController(TagConstants.X_REEF_ALIGNMENT_P, 0, 0); // Vertical movement
//         yController = new PIDController(TagConstants.Y_REEF_ALIGNMENT_P, 0, 0); // Horizontal movement
//         rotController = new PIDController(TagConstants.ROT_REEF_ALIGNMENT_P, 0, 0); // Rotation
//         this.isRightScore = isRightScore;
//         this.swerve = swerve;
//         this.visionSubsystem = visionSubsystem;
//         addRequirements(swerve, visionSubsystem);
//     }


//     @Override
//     public void initialize() {
//         this.stopTimer = new Timer();
//         this.stopTimer.start();
//         this.dontSeeTagTimer = new Timer();
//         this.dontSeeTagTimer.start();


//         rotController.setSetpoint(TagConstants.ROT_SETPOINT_REEF_ALIGNMENT);
//         rotController.setTolerance(TagConstants.ROT_TOLERANCE_REEF_ALIGNMENT);


//         xController.setSetpoint(TagConstants.X_SETPOINT_REEF_ALIGNMENT);
//         xController.setTolerance(TagConstants.X_TOLERANCE_REEF_ALIGNMENT);


//         yController.setSetpoint(TagConstants.Y_SETPOINT_REEF_ALIGNMENT);
//         yController.setTolerance(TagConstants.Y_TOLERANCE_REEF_ALIGNMENT);
//     }


//     @Override
//     public void execute() {
//         Optional<Pose3d> cameraPoseOptional = visionSubsystem.getCameraPose();
//         var result = camera.getLatestResult();


//         if (cameraPoseOptional.isPresent()) {
//             this.dontSeeTagTimer.reset();


//             PhotonTrackedTarget bestTarget = result.getBestTarget();
//             var target = bestTarget.getBestCameraToTarget();


//             Pose3d cameraPose = cameraPoseOptional.get();


//             // double x = cameraPose.getX();
//             // double y = cameraPose.getY();
//             // double rotation = cameraPose.getRotation().getZ();


//             // double[] positions = {target.getX(), 0 , target.getZ(), 0 , bestTarget.getYaw()};
//             // double xPosition = target.getZ();
//             // double yPosition = target.getY();
//             // double yaw = bestTarget.getYaw();


//             // double xSpeed = xController.calculate(xPosition);
//             // double ySpeed = -yController.calculate(yPosition);
//             // double rotValue = -rotController.calculate(yaw);


//             // SmartDashboard.putNumber("x", xSpeed);
//             // SmartDashboard.putNumber("y", y);
//             // SmartDashboard.putNumber("rotation", rotation);




//             //  Kesley: New AprilTag-based movement (commented out for now)
//              double targetYaw = cameraPose.getRotation().getZ();
//              double targetX = cameraPose.getX();


//              double xSpeed = isRightScore ? 0.5 : -0.5;   //worked but it was slow
//              double ySpeed = targetX * 0.2;
//              double rotValue = -targetYaw * 0.1;
           
//             // swerve.drive(
//             //     new Translation2d(yController.getError() < tagConstants.Y_TOLERANCE_REEF_ALIGNMENT ? xSpeed : 0, ySpeed),
//             //     rotValue, false);


//             swerve.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);


//             if (!rotController.atSetpoint() || !xController.atSetpoint() || !yController.atSetpoint()) {
//                 stopTimer.reset();
//             } else {
//                 swerve.drive(new Translation2d(), 0, false);
//                 System.out.println("AprilTag not detected or incorrect ID");
//             }


//             SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
//         }
//     }
   
//     @Override
//     public void end(boolean interrupted) {
//         swerve.drive(new Translation2d(), 0, false);
//     }


//     @Override
//     public boolean isFinished() {
//         return this.dontSeeTagTimer.hasElapsed(TagConstants.DONT_SEE_TAG_WAIT_TIME) ||
//             stopTimer.hasElapsed(TagConstants.POSE_VALIDATION_TIME);
//     }
// }