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

    //
    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, boolean AbsoluteEncoderReversed, double absoluteEncoderOffsetRadians, int abosoluteEncoderId) {
        this.absoluteEncoderOffsetRadians = absoluteEncoderOffsetRadians;
        this.AbsoluteEncoderReversed = AbsoluteEncoderReversed;
        this.AbsoluteEncoder = new AnalogInput(abosoluteEncoderId);
        
        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
        
        //
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.inverted(driveMotorReversed);
        driveConfig.encoder.positionConversionFactor(ModuleConstants.DRIVE_ENCODER_ROTATIONS_TO_METERS);
        driveConfig.encoder.velocityConversionFactor(ModuleConstants.DRIVE_ENCODER_ROTATIONS_TO_METERS_PER_SECOND);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        //
        SparkMaxConfig turningConfig = new SparkMaxConfig();
        turningConfig.inverted(turningMotorReversed);
        turningConfig.encoder.positionConversionFactor(ModuleConstants.TURNING_ENCODER_ROTATIONS_TO_RADIANS);
        turningConfig.encoder.velocityConversionFactor(ModuleConstants.TURNING_ENCODER_ROTATIONS_TO_RADIANS_PER_SECOND);
        turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        turningPIDController = new PIDController(ModuleConstants.TURNING_PID_PROPORTIONAL_GAIN, 0.0, 0.0);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

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
    
    //
    public double getAbsoluteEncoderRadians() {
        //
        double angle = AbsoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRadians;
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
    
    //
    public void setDesiredState(SwerveModuleState desiredState) {
        //
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        driveMotor.set(desiredState.speedMetersPerSecond / ModuleConstants.DRIVE_MOTOR_MAXIMUM_SPEED_METERS_PER_SECOND);
        turningMotor.set(turningPIDController.calculate(getTurningPosition(), desiredState.angle.getRadians()));
    }
}
