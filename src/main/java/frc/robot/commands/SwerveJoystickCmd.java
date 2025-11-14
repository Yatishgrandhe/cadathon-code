package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> feildOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

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
        this.xLimiter = new SlewRateLimiter(DriveConstants.TELEOP_MAXIMUM_ACCELERATION_UNITS_PER_SECOND);
        this.yLimiter = new SlewRateLimiter(DriveConstants.TELEOP_MAXIMUM_ACCELERATION_UNITS_PER_SECOND);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.TELEOP_MAXIMUM_ANGULAR_ACCELERATION_UNITS_PER_SECOND);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double xSpd = xSpdFunction.get();
        double ySpd = ySpdFunction.get();
        double turningSpd = turningSpdFunction.get();
        xSpd = Math.abs(xSpd) > OperatorConstants.JOYSTICK_DEADBAND_THRESHOLD ? xSpd : 0.0;
        ySpd = Math.abs(ySpd) > OperatorConstants.JOYSTICK_DEADBAND_THRESHOLD ? ySpd : 0.0;
        turningSpd = Math.abs(turningSpd) > OperatorConstants.JOYSTICK_DEADBAND_THRESHOLD ? turningSpd : 0.0;

        xSpd = xLimiter.calculate(xSpd)*DriveConstants.TELEOP_MAXIMUM_SPEED_METERS_PER_SECOND;
        ySpd = yLimiter.calculate(ySpd)*DriveConstants.TELEOP_MAXIMUM_SPEED_METERS_PER_SECOND;
        turningSpd = turningLimiter.calculate(turningSpd)*DriveConstants.TELEOP_MAXIMUM_ANGULAR_SPEED_RADIANS_PER_SECOND;  
        ChassisSpeeds chassisSpeeds;
        if (feildOrientedFunction.get()) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpd, ySpd, turningSpd, swerveSubsystem.getRotation2d());
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpd, ySpd, turningSpd);
        }
        SwerveModuleState[] moduleStates = DriveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
    }
    
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopmodules();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
