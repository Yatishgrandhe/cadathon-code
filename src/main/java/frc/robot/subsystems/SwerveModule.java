package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    //
    private final SparkMax driveMotor;
    private final SparkMax turningMotor;
    
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPIDController;

    private final AnalogInput AbsoluteEncoder;
    private final boolean AbsoluteEncoderReversed;
    private final double absoluteEncoderOffsetRadians;

    // Constructor takes many parameters to allow each module to be configured independently based on physical hardware differences
    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, boolean AbsoluteEncoderReversed, double absoluteEncoderOffsetRadians, int abosoluteEncoderId) {
        // Offset and reversed flags are stored as fields because they're needed every time we read the absolute encoder
        this.absoluteEncoderOffsetRadians = absoluteEncoderOffsetRadians;
        this.AbsoluteEncoderReversed = AbsoluteEncoderReversed;
        // Analog input is used for absolute encoder because it provides continuous position feedback even after power loss
        this.AbsoluteEncoder = new AnalogInput(abosoluteEncoderId);
        
        // Brushless motors are used because they're more efficient and reliable than brushed motors for swerve drive
        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
        
        // Drive motor configuration is separate from turning motor because they have different gear ratios and requirements
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        // Motor inversion is configured here to handle cases where motors are physically mounted backwards
        driveConfig.inverted(driveMotorReversed);
        // Conversion factors are set so encoder readings are automatically in meters and meters per second for easier use
        driveConfig.encoder.positionConversionFactor(ModuleConstants.DRIVE_ENCODER_ROTATIONS_TO_METERS);
        driveConfig.encoder.velocityConversionFactor(ModuleConstants.DRIVE_ENCODER_ROTATIONS_TO_METERS_PER_SECOND);
        // Reset and persist modes ensure configuration is saved to motor controller and survives power cycles
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // Turning motor configuration is separate because it needs different units (radians instead of meters)
        SparkMaxConfig turningConfig = new SparkMaxConfig();
        turningConfig.inverted(turningMotorReversed);
        // Radians conversion factors are used because WPILib's Rotation2d and kinematics use radians
        turningConfig.encoder.positionConversionFactor(ModuleConstants.TURNING_ENCODER_ROTATIONS_TO_RADIANS);
        turningConfig.encoder.velocityConversionFactor(ModuleConstants.TURNING_ENCODER_ROTATIONS_TO_RADIANS_PER_SECOND);
        turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // Encoders are retrieved from motors because they're built into the SparkMax controllers
        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        // PID controller is used for turning because it provides smooth, accurate angle control
        turningPIDController = new PIDController(ModuleConstants.TURNING_PID_PROPORTIONAL_GAIN, 0.0, 0.0);
        // Continuous input is enabled because angles wrap around (360° = 0°), so PID needs to know shortest path to target

        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        // Encoders are reset at startup to synchronize relative encoders with absolute encoder position
        resetEncoders();
    }
    
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }
    
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }
    
    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }
    
    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }
    
    // Absolute encoder reading is needed to get true wheel angle even after power loss, unlike relative encoders that reset
    public double getAbsoluteEncoderRadians() {
        // Voltage is normalized by 5V supply because absolute encoders output 0-5V proportional to angle
        double angle = AbsoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        // Multiply by 2π to convert normalized 0-1 value to full rotation in radians
        angle *= 2.0 * Math.PI;
        // Offset is subtracted to account for physical mounting position of encoder on the wheel
        angle -= absoluteEncoderOffsetRadians;
        // Reversal is applied if encoder is mounted backwards, ensuring consistent coordinate system
        return angle*(AbsoluteEncoderReversed ? -1.0 : 1.0);
    }
    
    public void resetEncoders() {
        driveEncoder.setPosition(0.0);
        turningEncoder.setPosition(getAbsoluteEncoderRadians());
    }
    
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }
    
    public void stop() {
        driveMotor.set(0.0);
        turningMotor.set(0.0);
    }
    
    // This method is called by the subsystem to set each module's speed and angle based on kinematics calculations
    public void setDesiredState(SwerveModuleState desiredState) {
        // Small speed threshold prevents unnecessary wheel turning when robot should be stopped, reducing wear
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        // State optimization finds shortest rotation path to target angle, preventing wheels from spinning 180° unnecessarily
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        // Speed is normalized by maximum to convert from meters/second to -1.0 to 1.0 motor output range
        driveMotor.set(desiredState.speedMetersPerSecond / ModuleConstants.DRIVE_MOTOR_MAXIMUM_SPEED_METERS_PER_SECOND);
        // PID controller calculates turning motor output to smoothly reach desired angle with minimal overshoot
        turningMotor.set(turningPIDController.calculate(getTurningPosition(), desiredState.angle.getRadians()));
    }
}
