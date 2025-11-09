package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

// SubsystemBase is extended to integrate with WPILib's command-based framework and get periodic callbacks
public class SwerveSubsystem extends SubsystemBase {
    // Four modules are created as fields because they need to persist and be accessed throughout the robot's lifetime
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.FRONT_LEFT_DRIVE_MOTOR_PORT,
        DriveConstants.FRONT_LEFT_TURNING_MOTOR_PORT,
        DriveConstants.FRONT_LEFT_DRIVE_ENCODER_REVERSED,
        DriveConstants.FRONT_LEFT_TURNING_ENCODER_REVERSED,
        DriveConstants.FRONT_LEFT_ABSOLUTE_ENCODER_REVERSED,
        DriveConstants.FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS,
        DriveConstants.FRONT_LEFT_ABSOLUTE_ENCODER_PORT);
    
    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_PORT,
        DriveConstants.FRONT_RIGHT_TURNING_MOTOR_PORT,
        DriveConstants.FRONT_RIGHT_DRIVE_ENCODER_REVERSED,
        DriveConstants.FRONT_RIGHT_TURNING_ENCODER_REVERSED,
        DriveConstants.FRONT_RIGHT_ABSOLUTE_ENCODER_REVERSED,
        DriveConstants.FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS,
        DriveConstants.FRONT_RIGHT_ABSOLUTE_ENCODER_PORT);
    
    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.BACK_LEFT_DRIVE_MOTOR_PORT,
        DriveConstants.BACK_LEFT_TURNING_MOTOR_PORT,
        DriveConstants.BACK_LEFT_DRIVE_ENCODER_REVERSED,
        DriveConstants.BACK_LEFT_TURNING_ENCODER_REVERSED,
        DriveConstants.BACK_LEFT_ABSOLUTE_ENCODER_REVERSED,
        DriveConstants.BACK_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS,
        DriveConstants.BACK_LEFT_ABSOLUTE_ENCODER_PORT);
    
    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.BACK_RIGHT_DRIVE_MOTOR_PORT,
        DriveConstants.BACK_RIGHT_TURNING_MOTOR_PORT,
        DriveConstants.BACK_RIGHT_DRIVE_ENCODER_REVERSED,
        DriveConstants.BACK_RIGHT_TURNING_ENCODER_REVERSED,
        DriveConstants.BACK_RIGHT_ABSOLUTE_ENCODER_REVERSED,
        DriveConstants.BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS,
        DriveConstants.BACK_RIGHT_ABSOLUTE_ENCODER_PORT);
    
    // Kinematics object is created here instead of using the constant to allow for potential runtime modifications
    private final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(DriveConstants.WHEEL_BASE_METERS / 2.0, DriveConstants.TRACK_WIDTH_METERS / 2.0),
        new Translation2d(DriveConstants.WHEEL_BASE_METERS / 2.0, -DriveConstants.TRACK_WIDTH_METERS / 2.0),
        new Translation2d(-DriveConstants.WHEEL_BASE_METERS / 2.0, DriveConstants.TRACK_WIDTH_METERS / 2.0),
        new Translation2d(-DriveConstants.WHEEL_BASE_METERS / 2.0, -DriveConstants.TRACK_WIDTH_METERS / 2.0)
    );
    
    // Heading is stored as a simple double because gyro integration hasn't been implemented yet, this is a placeholder
    private double heading = 0.0;

    // NavX gyro is used to get accurate robot heading for field-relative driving
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private SwerveDriveOdometry odometer = new SwerveDriveOdometry(
        swerveDriveKinematics, 
        new Rotation2d(0), 
        new SwerveModulePosition[] {
            new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getTurningPosition())),
            new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurningPosition())),
            new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurningPosition())),
            new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getTurningPosition()))
        }
    );
    
    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                ZeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }
    
    // ZeroHeading allows driver to reset field-relative coordinate system when robot is aligned with field
    public void ZeroHeading() {
        gyro.reset();
    }
    
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360.0);
    }
    
    // getRotation2d converts heading to Rotation2d because WPILib's field-relative functions require Rotation2d type
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometer(Pose2d pose) {
        odometer.resetPosition(
            getRotation2d(),
            new SwerveModulePosition[] {
                new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getTurningPosition())),
                new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurningPosition())),
                new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurningPosition())),
                new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getTurningPosition()))
            },
            pose
        );
    }

    @Override
    // Periodic runs every robot cycle to update SmartDashboard for driver visibility and debugging
    public void periodic() {
        odometer.update(
            getRotation2d(),
            new SwerveModulePosition[] {
                new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getTurningPosition())),
                new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurningPosition())),
                new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurningPosition())),
                new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getTurningPosition()))
            }
        );
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }
    
    // stopmodules is called when robot needs to stop immediately, such as when commands are interrupted
    public void stopmodules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
    
    // This method is the main interface for driving the robot, converting chassis-level commands to module-level commands
    public void setModuleStates(SwerveModuleState[] moduleStates) {
        // Desaturation prevents any module from exceeding maximum speed, maintaining proper swerve drive behavior
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, ModuleConstants.DRIVE_MOTOR_MAXIMUM_SPEED_METERS_PER_SECOND);
        // Each module gets its state from the array in the order defined by kinematics (front-left, front-right, back-left, back-right)
        frontLeft.setDesiredState(moduleStates[0]);
        frontRight.setDesiredState(moduleStates[1]);
        backLeft.setDesiredState(moduleStates[2]);
        backRight.setDesiredState(moduleStates[3]);
    }
}
