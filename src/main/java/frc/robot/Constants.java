package frc.robot;
//ALL OF THESE ARE CONSTANTS FOR THE ROBOT and Will Need to be Changed Based on CAD
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

// Constants class groups all configuration values together so they can be easily found and modified without searching through code
public final class Constants {
  // OperatorConstants groups driver input settings together for easy tuning of control behavior
  public static class OperatorConstants {
    // Port 0 is used because it's the default port for the primary driver station controller
    public static final int DRIVER_CONTROLLER_PORT = 0;
    // Port 0 matches the controller port for consistency
    public static final int DRIVER_JOYSTICK_PORT = 0;
    // Deadband threshold prevents small joystick drift from causing unwanted robot movement
    public static final double JOYSTICK_DEADBAND_THRESHOLD = 0.05;
    
    // Axis indices match standard joystick axis numbering (0=X, 1=Y, 2=rotation)
    public static final int DRIVER_JOYSTICK_X_AXIS = 0;
    public static final int DRIVER_JOYSTICK_Y_AXIS = 1;
    public static final int DRIVER_JOYSTICK_ROTATION_AXIS = 2;
    // Button 1 is used because it's easily accessible and commonly used for mode switching
    public static final int DRIVER_JOYSTICK_FIELD_ORIENTED_BUTTON_INDEX = 1;
  }

  // ModuleConstants groups swerve module hardware-specific values to keep them organized and easy to adjust
  public static class ModuleConstants {
    // Wheel diameter is converted to meters because WPILib kinematics requires metric units
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
    
    // Gear ratio is inverted (1/ratio) because we need output rotations per input rotation for encoder conversion
    public static final double DRIVE_MOTOR_GEAR_RATIO = 1.0 / 5.8462;
    
    // Turning gear ratio is inverted for the same reason as drive gear ratio
    public static final double TURNING_MOTOR_GEAR_RATIO = 1.0 / 18.0;
    
    // This conversion factor is calculated once to avoid recalculating during runtime for performance
    public static final double DRIVE_ENCODER_ROTATIONS_TO_METERS = DRIVE_MOTOR_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
    
    // Radians conversion uses 2π because one full rotation equals 2π radians
    public static final double TURNING_ENCODER_ROTATIONS_TO_RADIANS = 2.0 * TURNING_MOTOR_GEAR_RATIO * Math.PI;
    
    // Velocity conversions divide by 60 because encoders typically report rotations per minute, not per second
    public static final double DRIVE_ENCODER_ROTATIONS_TO_METERS_PER_SECOND = DRIVE_ENCODER_ROTATIONS_TO_METERS / 60.0;
    public static final double TURNING_ENCODER_ROTATIONS_TO_RADIANS_PER_SECOND = TURNING_ENCODER_ROTATIONS_TO_RADIANS / 60.0;
    
    // Proportional gain of 0.5 provides responsive turning without excessive oscillation
    public static final double TURNING_PID_PROPORTIONAL_GAIN = 0.5;
    
    // Maximum speed is set based on physical motor and gear limitations to prevent overdriving the hardware
    public static final double DRIVE_MOTOR_MAXIMUM_SPEED_METERS_PER_SECOND = 4.5;
  }

  public static class DriveConstants {
    // Track width and wheel base are converted to meters because WPILib requires metric units for kinematics
    public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(21);
    public static final double WHEEL_BASE_METERS = Units.inchesToMeters(25.5);

    // Kinematics object is created here as a constant because it's expensive to create and never changes
    public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0),
      new Translation2d(WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0),
      new Translation2d(-WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0),
      // Module positions are calculated from center of robot to match standard swerve drive coordinate system
      new Translation2d(-WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0)
    );
    
    // Motor ports are numbered sequentially (1-8) to make wiring and troubleshooting easier
    public static final int FRONT_LEFT_DRIVE_MOTOR_PORT = 1;
    public static final int FRONT_LEFT_TURNING_MOTOR_PORT = 2;
    // Encoder reversed flags are set to false initially and adjusted during physical testing if motors spin wrong direction
    public static final boolean FRONT_LEFT_DRIVE_ENCODER_REVERSED = false;
    public static final boolean FRONT_LEFT_TURNING_ENCODER_REVERSED = false;
    public static final boolean FRONT_LEFT_ABSOLUTE_ENCODER_REVERSED = false;
    // Absolute encoder ports are numbered 0-3 to match analog input ports on the roboRIO
    public static final int FRONT_LEFT_ABSOLUTE_ENCODER_PORT = 0;
    // Offset is set to 0.0 initially and calibrated during robot setup to account for encoder mounting position
    public static final double FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS = 0.0;
    // Front right module uses ports 3-4 to continue sequential numbering
    public static final int FRONT_RIGHT_DRIVE_MOTOR_PORT = 3;
    public static final int FRONT_RIGHT_TURNING_MOTOR_PORT = 4;
    public static final boolean FRONT_RIGHT_DRIVE_ENCODER_REVERSED = false;
    public static final boolean FRONT_RIGHT_TURNING_ENCODER_REVERSED = false;
    public static final boolean FRONT_RIGHT_ABSOLUTE_ENCODER_REVERSED = false;
    public static final int FRONT_RIGHT_ABSOLUTE_ENCODER_PORT = 1;
    public static final double FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS = 0.0;
    // Back left module uses ports 5-6 to continue sequential numbering
    public static final int BACK_LEFT_DRIVE_MOTOR_PORT = 5;
    public static final int BACK_LEFT_TURNING_MOTOR_PORT = 6;
    public static final boolean BACK_LEFT_DRIVE_ENCODER_REVERSED = false;
    public static final boolean BACK_LEFT_TURNING_ENCODER_REVERSED = false;
    public static final boolean BACK_LEFT_ABSOLUTE_ENCODER_REVERSED = false;
    public static final int BACK_LEFT_ABSOLUTE_ENCODER_PORT = 2;
    public static final double BACK_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS = 0.0;
    // Back right module uses ports 7-8 to complete sequential numbering
    public static final int BACK_RIGHT_DRIVE_MOTOR_PORT = 7;
    public static final int BACK_RIGHT_TURNING_MOTOR_PORT = 8;
    public static final boolean BACK_RIGHT_DRIVE_ENCODER_REVERSED = false;
    public static final boolean BACK_RIGHT_TURNING_ENCODER_REVERSED = false;
    public static final boolean BACK_RIGHT_ABSOLUTE_ENCODER_REVERSED = false;
    public static final int BACK_RIGHT_ABSOLUTE_ENCODER_PORT = 3;
    public static final double BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS = 0.0;
    // Teleop speed limits are set lower than maximum to provide safer, more controllable driving
    public static final double TELEOP_MAXIMUM_SPEED_METERS_PER_SECOND = 3.0;
    // Angular speed uses π radians per second to allow one full rotation per second, which feels natural to drivers
    public static final double TELEOP_MAXIMUM_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
    // Acceleration limits prevent sudden jerky movements that could damage hardware or make driving difficult
    public static final double TELEOP_MAXIMUM_ACCELERATION_UNITS_PER_SECOND = 3.0;
    public static final double TELEOP_MAXIMUM_ANGULAR_ACCELERATION_UNITS_PER_SECOND = Math.PI;
    // Autonomous speed limits are typically higher than teleop for faster path following
    public static final double AUTO_MAXIMUM_SPEED_METERS_PER_SECOND = 4.0;
    // Autonomous acceleration is set to allow smooth but responsive path following
    public static final double AUTO_MAXIMUM_ACCELERATION_METERS_PER_SECOND_SQUARED = 3.0;
  }

  // AutoConstants groups autonomous PID controller gains for trajectory following
  public static class AutoConstants {
    // X controller proportional gain for trajectory following (tuned for smooth path following)
    public static final double kpxContoller = 1.0;
    // Y controller proportional gain for trajectory following
    public static final double kpyContoller = 1.0;
    // Theta (rotation) controller proportional gain for trajectory following
    public static final double kpthetaContoller = 1.0;
    // Theta controller constraints define maximum velocity and acceleration for rotation
    public static final TrapezoidProfile.Constraints kpthetaContollerConstraints = 
        new TrapezoidProfile.Constraints(Math.PI, Math.PI);
  }
}
