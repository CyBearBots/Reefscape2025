// package frc.robot.commands;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystems.swervedrive.SwerveSubsystem;
// import frc.robot.subsystems.VisionSubsystem;

// import java.util.Optional;

// public class BargeAlignCommand extends Command {
//     private final SwerveSubsystem swerve;
//     private final VisionSubsystem visionSubsystem;
//     private Pose2d distFromTag;  
//     private final PIDController xController = new PIDController(Constants.SwerveConstants.kPX, 0, 0);
//     private final PIDController yController = new PIDController(Constants.SwerveConstants.kPY, 0, 0);
//     private final PIDController thetaController = new PIDController(Constants.SwerveConstants.kPTheta, 0, 0);

//     public BargeAlignCommand(SwerveSubsystem swerve, VisionSubsystem visionSubsystem) {
//         this.swerve = swerve;
//         this.visionSubsystem = visionSubsystem;
//     }

//     @Override
//     public void initialize() {}

//     @Override
//     public void execute() {
//         Optional<Pose2d> cameraPoseOpt = visionSubsystem.getCameraPose().map(pose3d -> new Pose2d(pose3d.getTranslation().getX(), pose3d.getTranslation().getY(), pose3d.getRotation().toRotation2d()));

//         if (cameraPoseOpt.isPresent()) {
//             Pose2d currentPose = cameraPoseOpt.get();
//             distFromTag = currentPose;

//             double xPower = MathUtil.clamp(xController.calculate(currentPose.getX(), Constants.VisionConstants.bargeAlignmentX), -0.3, 0.3);
//             double yPower = MathUtil.clamp(yController.calculate(currentPose.getY(), Constants.VisionConstants.bargeAlignmentY), -0.3, 0.3);
//             double thetaPower = thetaController.calculate(currentPose.getRotation().getRadians(), Constants.VisionConstants.bargeThetaAlignment);

//             SmartDashboard.putNumber("Subsystem/Vision/actualAlignmentX", currentPose.getX());
//             SmartDashboard.putNumber("Subsystem/Vision/actualAlignmentY", currentPose.getY());
//             SmartDashboard.putNumber("Subsystem/Vision/actualAlignmentTheta", currentPose.getRotation().getRadians());
//             SmartDashboard.putNumber("Subsystem/Vision/xPOut", xPower);
//             SmartDashboard.putNumber("Subsystem/Vision/yPOut", yPower);
//             SmartDashboard.putNumber("Subsystem/Vision/thetaOut", thetaPower);

//             swerve.drive(new ChassisSpeeds(-yPower, xPower, thetaPower));
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         Optional<Pose2d> cameraPoseOpt = visionSubsystem.getCameraPose()
//             .map(pose3d -> new Pose2d(pose3d.getTranslation().getX(), pose3d.getTranslation().getY(), pose3d.getRotation().toRotation2d()));

//         if (cameraPoseOpt.isEmpty()) {
//             return false;
//         }

//         Pose2d currentPose = cameraPoseOpt.get();
//         distFromTag = currentPose;

//         return (Math.abs(distFromTag.getX() - Constants.VisionConstants.bargeAlignmentX) < Constants.VisionConstants.xTolerance) 
//             && (Math.abs(distFromTag.getY() - Constants.VisionConstants.bargeAlignmentY) < Constants.VisionConstants.yTolerance)
//             && (Math.abs(distFromTag.getRotation().getRadians() - Constants.VisionConstants.bargeThetaAlignment) < Constants.VisionConstants.thetaTolerance); 
//     }
// }
