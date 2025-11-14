package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TheForce_Hanging;
import frc.robot.subsystems.ColorWheelSubsystem;
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
import frc.robot.commands.HangingCmd;
import frc.robot.commands.RotateColorWheelCmd;
import frc.robot.commands.GoToColorCmd;
import frc.robot.subsystems.DetectedColor;

public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final TheForce_Hanging hangingSubsystem = new TheForce_Hanging();
  private final ColorWheelSubsystem colorWheelSubsystem = new ColorWheelSubsystem();
  private final Joystick driverJoystick = new Joystick(OperatorConstants.DRIVER_JOYSTICK_PORT);
  
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem, 
      () -> -driverJoystick.getRawAxis(OperatorConstants.DRIVER_JOYSTICK_Y_AXIS),
      () -> driverJoystick.getRawAxis(OperatorConstants.DRIVER_JOYSTICK_X_AXIS), 
      () -> driverJoystick.getRawAxis(OperatorConstants.DRIVER_JOYSTICK_ROTATION_AXIS), 
      () -> !driverJoystick.getRawButton(OperatorConstants.DRIVER_JOYSTICK_FIELD_ORIENTED_BUTTON_INDEX)));
    hangingSubsystem.setDefaultCommand(new HangingCmd(
        hangingSubsystem,
        () -> driverJoystick.getRawButton(3),
        () -> driverJoystick.getRawButton(4),
        () -> driverJoystick.getRawButton(5),
        () -> driverJoystick.getRawButton(6),
        () -> driverJoystick.getRawButton(7),
        () -> driverJoystick.getRawButton(8)
    ));    
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driverJoystick, 2).onTrue(swerveSubsystem.runOnce(() -> swerveSubsystem.ZeroHeading()));
    new JoystickButton(driverJoystick, 9).onTrue(
        new RotateColorWheelCmd(colorWheelSubsystem)
    );
    
    // Button 10 = Go to Red (example)
    new JoystickButton(driverJoystick, 10).onTrue(
        new GoToColorCmd(colorWheelSubsystem, DetectedColor.RED)
    );
  }

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
    return new SequentialCommandGroup(
        new InstantCommand(()->swerveSubsystem.resetOdometer(trajectory.getInitialPose())),
        swervecontrollercommand,
        new InstantCommand(()->swerveSubsystem.stopmodules())
    );
  }
}
