package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

// This command extends Command to integrate with WPILib's command-based framework for consistent execution
public class SwerveJoystickCmd extends Command {
    // Subsystem is stored to access swerve drive functionality
    private final SwerveSubsystem swerveSubsystem;
    // Suppliers are used instead of direct values to allow joystick input to be read fresh each execute() call
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    // Boolean supplier allows field-oriented mode to be toggled dynamically during command execution
    private final Supplier<Boolean> feildOrientedFunction;
    // Slew rate limiters smooth out joystick input to prevent sudden jerky movements that could damage hardware
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    // Constructor takes suppliers as parameters to decouple this command from specific input devices, making it testable
    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
                             Supplier<Double> xSpdFunction,
                             Supplier<Double> ySpdFunction,
                             Supplier<Double> turningSpdFunction,
                             Supplier<Boolean> feildOrientedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.feildOrientedFunction = feildOrientedFunction;
        // Slew rate limiters are created with acceleration limits to smooth out control inputs
        this.xLimiter = new SlewRateLimiter(DriveConstants.TELEOP_MAXIMUM_ACCELERATION_UNITS_PER_SECOND);
        this.yLimiter = new SlewRateLimiter(DriveConstants.TELEOP_MAXIMUM_ACCELERATION_UNITS_PER_SECOND);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.TELEOP_MAXIMUM_ANGULAR_ACCELERATION_UNITS_PER_SECOND);
        // addRequirements prevents other commands from using the swerve subsystem simultaneously
        addRequirements(swerveSubsystem);
    }

    @Override
    // Empty because no initialization needed - command runs continuously as default command
    public void initialize() {
    }

    @Override
    // Execute runs every robot cycle to continuously read joystick input and drive the robot
    public void execute() {
        // Raw joystick values are read fresh each cycle to get real-time input
        double xSpd = xSpdFunction.get();
        double ySpd = ySpdFunction.get();
        double turningSpd = turningSpdFunction.get();
        // Deadband filtering prevents small joystick drift from causing unwanted robot movement when joystick is centered
        xSpd = Math.abs(xSpd) > OperatorConstants.JOYSTICK_DEADBAND_THRESHOLD ? xSpd : 0.0;
        ySpd = Math.abs(ySpd) > OperatorConstants.JOYSTICK_DEADBAND_THRESHOLD ? ySpd : 0.0;
        turningSpd = Math.abs(turningSpd) > OperatorConstants.JOYSTICK_DEADBAND_THRESHOLD ? turningSpd : 0.0;

        // Slew rate limiting smooths acceleration to prevent sudden speed changes that could damage motors or make driving difficult
        xSpd = xLimiter.calculate(xSpd)*DriveConstants.TELEOP_MAXIMUM_SPEED_METERS_PER_SECOND;
        ySpd = yLimiter.calculate(ySpd)*DriveConstants.TELEOP_MAXIMUM_SPEED_METERS_PER_SECOND;
        turningSpd = turningLimiter.calculate(turningSpd)*DriveConstants.TELEOP_MAXIMUM_ANGULAR_SPEED_RADIANS_PER_SECOND;  
        // Chassis speeds are created differently based on mode because field-oriented requires robot heading for coordinate transformation
        ChassisSpeeds chassisSpeeds;
        if (feildOrientedFunction.get()) {
            // Field-relative speeds are used when button is pressed to make driving intuitive regardless of robot orientation
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpd, ySpd, turningSpd, swerveSubsystem.getRotation2d());
        } else {
            // Robot-relative speeds are used by default for direct control matching joystick orientation
            chassisSpeeds = new ChassisSpeeds(xSpd, ySpd, turningSpd);
        }
        // Kinematics converts chassis speeds to individual module states because each swerve module needs its own speed and angle
        SwerveModuleState[] moduleStates = DriveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
    }
    
    @Override
    // Stop is called when command ends to ensure robot doesn't continue moving after command is interrupted or cancelled
    public void end(boolean interrupted) {
        swerveSubsystem.stopmodules();
    }
    
    @Override
    // Returns false because this is a default command that should run continuously, not finish on its own
    public boolean isFinished() {
        return false;
    }
}
