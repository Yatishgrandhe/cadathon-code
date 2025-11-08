package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
  //
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int DRIVER_JOYSTICK_PORT = 0;
    public static final double JOYSTICK_DEADBAND_THRESHOLD = 0.05;
    
    public static final int DRIVER_JOYSTICK_X_AXIS = 0;
    public static final int DRIVER_JOYSTICK_Y_AXIS = 1;
    public static final int DRIVER_JOYSTICK_ROTATION_AXIS = 2;
    public static final int DRIVER_JOYSTICK_FIELD_ORIENTED_BUTTON_INDEX = 1;
  }

  //
  public static class ModuleConstants {
    //
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
    
    public static final double DRIVE_MOTOR_GEAR_RATIO = 1.0 / 5.8462;
    
    public static final double TURNING_MOTOR_GEAR_RATIO = 1.0 / 18.0;
    
    public static final double DRIVE_ENCODER_ROTATIONS_TO_METERS = DRIVE_MOTOR_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
    
    public static final double TURNING_ENCODER_ROTATIONS_TO_RADIANS = 2.0 * TURNING_MOTOR_GEAR_RATIO * Math.PI;
    
    public static final double DRIVE_ENCODER_ROTATIONS_TO_METERS_PER_SECOND = DRIVE_ENCODER_ROTATIONS_TO_METERS / 60.0;
    public static final double TURNING_ENCODER_ROTATIONS_TO_RADIANS_PER_SECOND = TURNING_ENCODER_ROTATIONS_TO_RADIANS / 60.0;
    
    public static final double TURNING_PID_PROPORTIONAL_GAIN = 0.5;
    
    public static final double DRIVE_MOTOR_MAXIMUM_SPEED_METERS_PER_SECOND = 4.5;
  }

  public static class DriveConstants {
    public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(21);
    public static final double WHEEL_BASE_METERS = Units.inchesToMeters(25.5);

    //
    public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0),
      new Translation2d(WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0),
      new Translation2d(-WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0),
      new Translation2d(-WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0)
    );
    
    public static final int FRONT_LEFT_DRIVE_MOTOR_PORT = 1;
    public static final int FRONT_LEFT_TURNING_MOTOR_PORT = 2;
    public static final boolean FRONT_LEFT_DRIVE_ENCODER_REVERSED = false;
    public static final boolean FRONT_LEFT_TURNING_ENCODER_REVERSED = false;
    public static final boolean FRONT_LEFT_ABSOLUTE_ENCODER_REVERSED = false;
    public static final int FRONT_LEFT_ABSOLUTE_ENCODER_PORT = 0;
    public static final double FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS = 0.0;
    //
    public static final int FRONT_RIGHT_DRIVE_MOTOR_PORT = 3;
    public static final int FRONT_RIGHT_TURNING_MOTOR_PORT = 4;
    public static final boolean FRONT_RIGHT_DRIVE_ENCODER_REVERSED = false;
    public static final boolean FRONT_RIGHT_TURNING_ENCODER_REVERSED = false;
    public static final boolean FRONT_RIGHT_ABSOLUTE_ENCODER_REVERSED = false;
    public static final int FRONT_RIGHT_ABSOLUTE_ENCODER_PORT = 1;
    public static final double FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS = 0.0;
    //
    public static final int BACK_LEFT_DRIVE_MOTOR_PORT = 5;
    public static final int BACK_LEFT_TURNING_MOTOR_PORT = 6;
    public static final boolean BACK_LEFT_DRIVE_ENCODER_REVERSED = false;
    public static final boolean BACK_LEFT_TURNING_ENCODER_REVERSED = false;
    public static final boolean BACK_LEFT_ABSOLUTE_ENCODER_REVERSED = false;
    public static final int BACK_LEFT_ABSOLUTE_ENCODER_PORT = 2;
    public static final double BACK_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS = 0.0;
    //
    public static final int BACK_RIGHT_DRIVE_MOTOR_PORT = 7;
    public static final int BACK_RIGHT_TURNING_MOTOR_PORT = 8;
    public static final boolean BACK_RIGHT_DRIVE_ENCODER_REVERSED = false;
    public static final boolean BACK_RIGHT_TURNING_ENCODER_REVERSED = false;
    public static final boolean BACK_RIGHT_ABSOLUTE_ENCODER_REVERSED = false;
    public static final int BACK_RIGHT_ABSOLUTE_ENCODER_PORT = 3;
    public static final double BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS = 0.0;
    //
    public static final double TELEOP_MAXIMUM_SPEED_METERS_PER_SECOND = 3.0;
    public static final double TELEOP_MAXIMUM_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
    public static final double TELEOP_MAXIMUM_ACCELERATION_UNITS_PER_SECOND = 3.0;
    public static final double TELEOP_MAXIMUM_ANGULAR_ACCELERATION_UNITS_PER_SECOND = Math.PI;
  }
}
