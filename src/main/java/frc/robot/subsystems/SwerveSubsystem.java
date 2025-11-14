package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveSubsystem extends SubsystemBase {
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
    
    private final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(DriveConstants.WHEEL_BASE_METERS / 2.0, DriveConstants.TRACK_WIDTH_METERS / 2.0),
        new Translation2d(DriveConstants.WHEEL_BASE_METERS / 2.0, -DriveConstants.TRACK_WIDTH_METERS / 2.0),
        new Translation2d(-DriveConstants.WHEEL_BASE_METERS / 2.0, DriveConstants.TRACK_WIDTH_METERS / 2.0),
        new Translation2d(-DriveConstants.WHEEL_BASE_METERS / 2.0, -DriveConstants.TRACK_WIDTH_METERS / 2.0)
    );
    
    private double heading = 0.0;

    private AHRS gyro;

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
        if (RobotBase.isReal()) {
            try {
                gyro = new AHRS(SPI.Port.kMXP);
            } catch (Throwable e) {
                System.out.println("Warning: navX initialization failed: " + e.getMessage());
                gyro = null;
            }
        } else {
            gyro = null;
            System.out.println("navX disabled in simulation mode");
        }
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                ZeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }
    
    public void ZeroHeading() {
        if (gyro != null) {
            gyro.reset();
        }
    }
    
    public double getHeading() {
        if (gyro != null) {
            return Math.IEEEremainder(gyro.getAngle(), 360.0);
        }
        return 0.0;
    }
    
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
    
    public void stopmodules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
    
    public void setModuleStates(SwerveModuleState[] moduleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, ModuleConstants.DRIVE_MOTOR_MAXIMUM_SPEED_METERS_PER_SECOND);
        frontLeft.setDesiredState(moduleStates[0]);
        frontRight.setDesiredState(moduleStates[1]);
        backLeft.setDesiredState(moduleStates[2]);
        backRight.setDesiredState(moduleStates[3]);
    }
}
