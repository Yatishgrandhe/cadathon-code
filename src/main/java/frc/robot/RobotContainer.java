package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

// RobotContainer centralizes all robot wiring to keep subsystem and command setup organized in one place
public class RobotContainer {
  // Subsystem is created as a field so it persists for the robot's lifetime and can be shared with commands
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  
  // Joystick is created here to be reused by multiple commands and button bindings
  private final Joystick driverJoystick = new Joystick(OperatorConstants.DRIVER_JOYSTICK_PORT);
  
  // Constructor sets up default commands and button bindings to ensure robot is ready immediately after creation
  public RobotContainer() {
    // Default command ensures swerve drive always responds to joystick input, even when no other commands are running
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem, 
      // Y axis is negated because joystick forward is typically negative values, but we want forward to be positive
      () -> -driverJoystick.getRawAxis(OperatorConstants.DRIVER_JOYSTICK_Y_AXIS),
      // X axis uses raw value because joystick coordinate system matches robot coordinate system
      () -> driverJoystick.getRawAxis(OperatorConstants.DRIVER_JOYSTICK_X_AXIS), 
      // Rotation axis uses raw value for direct control
      () -> driverJoystick.getRawAxis(OperatorConstants.DRIVER_JOYSTICK_ROTATION_AXIS), 
      // Button is negated because we want field-oriented when button is NOT pressed (typical joystick behavior)
      () -> !driverJoystick.getRawButton(OperatorConstants.DRIVER_JOYSTICK_FIELD_ORIENTED_BUTTON_INDEX)));
    configureBindings();
  }

  // Separate method keeps button bindings organized and makes it easy to add more bindings later
  private void configureBindings() {
    // Button 2 is used for zeroing heading because it's easily accessible and not used for driving
    new JoystickButton(driverJoystick, 2).onTrue(swerveSubsystem.runOnce(() -> swerveSubsystem.ZeroHeading()));
  }

  // Returns null because autonomous routine hasn't been implemented yet, but method exists for future use
  public Command getAutonomousCommand() {
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(DriveConstants.AUTO_MAXIMUM_SPEED_METERS_PER_SECOND, DriveConstants.AUTO_MAXIMUM_ACCELERATION_METERS_PER_SECOND_SQUARED).setKinematics(DriveConstants.SWERVE_DRIVE_KINEMATICS);
    
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(1, 0), new Translation2d(1, -1)),
        new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
        trajectoryConfig
    );
    
    PIDController xController = new PIDController(AutoConstants.kpxContoller, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kpyContoller, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kpthetaContoller, 0, 0, AutoConstants.kpthetaContollerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    SwerveControllerCommand swervecontrollercommand = new SwerveControllerCommand(trajectory,swerveSubsystem::getPose, DriveConstants.SWERVE_DRIVE_KINEMATICS, xController, yController, thetaController, swerveSubsystem::setModuleStates, swerveSubsystem);
    return new SequentialCommandGroup(new InstantCommand(()->swerveSubsystem.resetOdometer(trajectory.getInitialPose())), new InstantCommand(()->swerveSubsystem.stopmodules()));
  }
}
