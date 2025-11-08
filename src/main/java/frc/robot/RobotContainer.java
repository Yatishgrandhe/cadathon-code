package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  //
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  
  private final Joystick driverJoystick = new Joystick(OperatorConstants.DRIVER_JOYSTICK_PORT);
  
  //
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem, 
      () -> -driverJoystick.getRawAxis(OperatorConstants.DRIVER_JOYSTICK_Y_AXIS),
      () -> driverJoystick.getRawAxis(OperatorConstants.DRIVER_JOYSTICK_X_AXIS), 
      () -> driverJoystick.getRawAxis(OperatorConstants.DRIVER_JOYSTICK_ROTATION_AXIS), 
      () -> !driverJoystick.getRawButton(OperatorConstants.DRIVER_JOYSTICK_FIELD_ORIENTED_BUTTON_INDEX)));
    configureBindings();
  }

  //
  private void configureBindings() {
    //
    new JoystickButton(driverJoystick, 2).onTrue(swerveSubsystem.runOnce(() -> swerveSubsystem.ZeroHeading()));
  }

  //
  public Command getAutonomousCommand() {
    return null;
  }
}
