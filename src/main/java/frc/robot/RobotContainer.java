// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

//import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import frc.robot.Constants.DriveConstants;
//import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;

import frc.robot.subsystems.ArmSubsystem;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
//import frc.robot.subsystems.MotorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.intakes.AlgaeIntakeSubsystem;
import frc.robot.subsystems.intakes.CoralIntakeSubsystem;
// import frc.robot.commands.CageDownCommand;
// import frc.robot.commands.CageLiftCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralPivotCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorManualCommand;
import frc.robot.commands.IntakeBallCommand;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.SetArmPositionCommand;
import frc.robot.commands.FunnelCommand;
import frc.robot.commands.SetArmZeroCommand;
import frc.robot.commands.SetElevatorZeroCommand;
//import frc.robot.commands.AlignToReef;
//import frc.robot.commands.BargeAlignCommand;
//import frc.robot.commands.ReefAlignCommand;
import frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(ElevatorConstants.elevatorMotor1Id, ElevatorConstants.elevatorMotor2Id);     
  private final ArmSubsystem armSubsystem = new ArmSubsystem(CoralPivotConstants.coralPivotMotor1Id);
  private final AlgaeIntakeSubsystem algaeIntakeSubsystem = new AlgaeIntakeSubsystem(AlgaeIntakeConstants.algaeLeftMotorId);
  private final CoralIntakeSubsystem coralIntakeSubsystem = new CoralIntakeSubsystem(CoralIntakeConstants.coralIntakeMotorId);
  private final FunnelSubsystem funnelSubsystem = new FunnelSubsystem(FunnelConstants.funnelMotorId);
  //private final VisionSubsystem visionSubsystem = new VisionSubsystem();

  // CageDownCommand cageDownCommand = new CageDownCommand();
  // CageLiftCommand cageLiftCommand = new CageLiftCommand();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);

  CommandXboxController buttonController = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)//Controls foward and backwards
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX() * 1)//.withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  // SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(false)
  //                                                            .allianceRelativeControl(true); // false true and opposite for robot oriented

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {

    buttonController.povUp().onTrue(
      new SequentialCommandGroup(
        new ElevatorCommand(elevatorSubsystem, ElevatorConstants.L4[0]),
        new SetArmPositionCommand(armSubsystem, ElevatorConstants.L4[1])
      )
    );    // Top D-pad

    buttonController.povRight().onTrue(
      new SequentialCommandGroup(
        // new SetArmPositionCommand(armSubsystem, ElevatorConstants.L3[1]),
        // new ElevatorCommand(elevatorSubsystem, ElevatorConstants.L3[0])
        new ElevatorCommand(elevatorSubsystem, ElevatorConstants.LB[0]),
        new SetArmPositionCommand(armSubsystem, ElevatorConstants.LB[1])
      )
    );    // Right D-pad

    buttonController.povLeft().onTrue(
      new SequentialCommandGroup(
        new SetArmPositionCommand(armSubsystem, ElevatorConstants.L2[1]),
        new ElevatorCommand(elevatorSubsystem, ElevatorConstants.L2[0])
      )
    );    // Left D-pad

    buttonController.povDown().onTrue(
      new SequentialCommandGroup(
        new ElevatorCommand(elevatorSubsystem, ElevatorConstants.L1[0]),
        new SetArmPositionCommand(armSubsystem, ElevatorConstants.L1[1])
      )
    );    // Down D-pad

    buttonController.rightStick().onTrue(
      new SequentialCommandGroup(
        new ElevatorCommand(elevatorSubsystem, ElevatorConstants.LP[0]),
        new SetArmPositionCommand(armSubsystem, ElevatorConstants.LP[1])
      )
    );

    buttonController.leftStick().onTrue(
      new SequentialCommandGroup(
        new SetArmPositionCommand(armSubsystem, ElevatorConstants.L0[1]),
        new ElevatorCommand(elevatorSubsystem, ElevatorConstants.L0[0])
      )
    );

    buttonController.rightTrigger().whileTrue(new CoralPivotCommand(armSubsystem, CoralPivotConstants.speedUp)); // RT Held
    buttonController.leftTrigger().whileTrue(new CoralPivotCommand(armSubsystem, -CoralPivotConstants.speedDown)); // LT Held

    // buttonController.rightStick().onTrue(
    //   new SequentialCommandGroup(
    //     new ElevatorCommand(elevatorSubsystem, ElevatorConstants.L3),
    //     new SetArmPositionCommand(armSubsystem, ElevatorConstants.LFAngle),
    //     new FunnelCommand(funnelSubsystem, FunnelConstants.funnelSpeed, true),
    //     new ElevatorCommand(elevatorSubsystem, ElevatorConstants.LF),
    //     new CoralIntakeCommand(coralIntakeSubsystem, CoralIntakeConstants.intakeSpeed).withTimeout(2)
    //     // new SetArmPositionCommand(armSubsystem, ElevatorConstants.L0Angle),
    //     // new ElevatorCommand(elevatorSubsystem, ElevatorConstants.L3)
    //   )  
    // ); // A Pressed might change to toggleOnTrue

    buttonController.a().whileTrue(new CoralIntakeCommand(coralIntakeSubsystem, CoralIntakeConstants.intakeSpeed)); // left bumper
    buttonController.b().whileTrue(new CoralIntakeCommand(coralIntakeSubsystem, CoralIntakeConstants.ejectSpeed)); // B Pressed

    buttonController.x().whileTrue(new IntakeBallCommand(algaeIntakeSubsystem, AlgaeIntakeConstants.intakeSpeed)); // X HELD
    buttonController.y().whileTrue(new IntakeBallCommand(algaeIntakeSubsystem, AlgaeIntakeConstants.ejectSpeed)); // Y HELD

    buttonController.back().toggleOnTrue(new FunnelCommand(funnelSubsystem, FunnelConstants.funnelSpeed));  // back pressed
    buttonController.button(8).onTrue(new SequentialCommandGroup(
      new SetArmZeroCommand(armSubsystem),
      new SetElevatorZeroCommand(elevatorSubsystem)
    ));

    buttonController.rightBumper().whileTrue(new ElevatorManualCommand(elevatorSubsystem, ElevatorConstants.liftMotor1SpeedUp));  // right bumper
    buttonController.leftBumper().whileTrue(new ElevatorManualCommand(elevatorSubsystem, -ElevatorConstants.liftMotor1SpeedDown)); // left bumper

    // elevatorSubsystem.setDefaultCommand(
    //   new ElevatorManualCommand(elevatorSubsystem, () -> buttonController.getLeftY()) // FYI: - value means up on joystick
    // );

    //driverXbox.y().onTrue(new ProcessorDriveCommand(visionSubsystem, drivebase));
    //driverXbox.povRight().onTrue(new AlignToReef(true, drivebase, visionSubsystem).withTimeout(7)); //Right D-Pad -- configure this to right reef pipe
    //driverXbox.povLeft().onTrue(new AlignToReef(false, drivebase, visionSubsystem).withTimeout(7)); //Left D-Pad -- configure this to left reef pipe

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    // NamedCommands.registerCommand("CoralDropOffL2", new SequentialCommandGroup(
    //   new ElevatorCommand(elevatorSubsystem, ElevatorConstants.L3),
    //   new SetArmPositionCommand(armSubsystem, ElevatorConstants.LFAngle),
    //   new FunnelCommand(funnelSubsystem, FunnelConstants.funnelSpeed, true),
    //   new ElevatorCommand(elevatorSubsystem, ElevatorConstants.LF),
    //   new CoralIntakeCommand(coralIntakeSubsystem, CoralIntakeConstants.intakeSpeed, true),
    //   new ElevatorCommand(elevatorSubsystem, ElevatorConstants.L3),
    //   new SetArmPositionCommand(armSubsystem, ElevatorConstants.L0Angle),
    //   new ElevatorCommand(elevatorSubsystem, ElevatorConstants.L0),
    //   new SetArmPositionCommand(armSubsystem, ElevatorConstants.L2Angle),
    //   new ElevatorCommand(elevatorSubsystem, ElevatorConstants.L2),
    //   new CoralIntakeCommand(coralIntakeSubsystem, CoralIntakeConstants.ejectSpeed, true),
    //   new ElevatorCommand(elevatorSubsystem, ElevatorConstants.L3),
    //   new SetArmPositionCommand(armSubsystem, ElevatorConstants.LFAngle)
    // ));

    // NamedCommands.registerCommand("CoralDropOffL4", new SequentialCommandGroup(
    //   new ElevatorCommand(elevatorSubsystem, ElevatorConstants.L3),
    //   new SetArmPositionCommand(armSubsystem, ElevatorConstants.LFAngle),
    //   new FunnelCommand(funnelSubsystem, FunnelConstants.funnelSpeed, true),
    //   new ElevatorCommand(elevatorSubsystem, ElevatorConstants.LF),
    //   new CoralIntakeCommand(coralIntakeSubsystem, CoralIntakeConstants.intakeSpeed, true),
    //   new ElevatorCommand(elevatorSubsystem, ElevatorConstants.L4),
    //   new SetArmPositionCommand(armSubsystem, ElevatorConstants.LFAngle),
    //   // lower elevator here 
    //   new CoralIntakeCommand(coralIntakeSubsystem, CoralIntakeConstants.ejectSpeed, true),
    //   new ElevatorCommand(elevatorSubsystem, ElevatorConstants.L3),
    //   new SetArmPositionCommand(armSubsystem, ElevatorConstants.LFAngle)
    // ));

    NamedCommands.registerCommand("SetZeroPositionArm", new SetArmZeroCommand(armSubsystem));
    NamedCommands.registerCommand("SetZeroPositionElevator", new SetElevatorZeroCommand(elevatorSubsystem));

    NamedCommands.registerCommand("ArmAngleL0", new SetArmPositionCommand(armSubsystem, ElevatorConstants.L0[1]));
    NamedCommands.registerCommand("ElevatorPositionL0", new ElevatorCommand(elevatorSubsystem, ElevatorConstants.L0[0]));

    NamedCommands.registerCommand("ArmAngleL2", new SetArmPositionCommand(armSubsystem, ElevatorConstants.L2[1]));
    NamedCommands.registerCommand("ElevatorPositionL2", new ElevatorCommand(elevatorSubsystem, ElevatorConstants.L2[0]));

    NamedCommands.registerCommand("ArmAngleL4", new SetArmPositionCommand(armSubsystem, ElevatorConstants.L4[1]));
    NamedCommands.registerCommand("ArmAngleL4Place", new SetArmPositionCommand(armSubsystem, ElevatorConstants.L4[2]));
    NamedCommands.registerCommand("ElevatorPositionL4", new ElevatorCommand(elevatorSubsystem, ElevatorConstants.L4[0]));

    NamedCommands.registerCommand("CoralEject", new CoralIntakeCommand(coralIntakeSubsystem, CoralIntakeConstants.ejectSpeed).withTimeout(1));

    NamedCommands.registerCommand("ResetGyro", (Commands.runOnce(drivebase::zeroGyroWithAlliance)));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);

    // Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
    //   () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //   () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //   () -> driverXbox.getRightX(),
    //   () -> driverXbox.getRightY()
    // );

    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    // Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // driveFieldOrientedAnglularVelocity driveFieldOrientedDirectAngle
    }

    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above! driveFieldOrientedAnglularVelocity

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyroWithAlliance)));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      //driverXbox.start().whileTrue(Commands.none());
     // driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());

      driverXbox.povLeft().whileTrue(drivebase.alignWithReef(() -> drivebase.vision.getRobotInTagSpace(), () -> driverXbox.getLeftY(), () -> drivebase.vision.getLatestID(), true));
      driverXbox.povRight().whileTrue(drivebase.alignWithReef(() -> drivebase.vision.getRobotInTagSpace(), () -> driverXbox.getLeftY(), () -> drivebase.vision.getLatestID(), false));

      driverXbox.back().toggleOnTrue(new FunnelCommand(funnelSubsystem, -FunnelConstants.funnelSpeed));  // back pressed

    }  
    // drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("DL4Auto"); // TaxiAuto, DL4Auto, EL4Auto (not done)
  }
 
  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
