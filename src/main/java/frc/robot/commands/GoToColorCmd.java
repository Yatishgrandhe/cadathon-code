package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ColorWheelSubsystem;
import frc.robot.subsystems.DetectedColor;

public class GoToColorCmd extends Command {
    private final ColorWheelSubsystem colorWheelSubsystem;
    private final DetectedColor targetFMSColor; // Color FMS wants
    private DetectedColor targetSensorColor;    // Color sensor should see
    
    public GoToColorCmd(ColorWheelSubsystem colorWheelSubsystem, DetectedColor fmsColor) {
        this.colorWheelSubsystem = colorWheelSubsystem;
        this.targetFMSColor = fmsColor;
        addRequirements(colorWheelSubsystem);
    }
    @Override
    public void initialize() {
        targetSensorColor = colorWheelSubsystem.getTargetSensorColor(targetFMSColor);
        colorWheelSubsystem.spinWheel(0.25);
    }
    @Override
    public void execute() {
    }
    @Override
    public boolean isFinished() {
        return colorWheelSubsystem.getCurrentColor() == targetSensorColor;
    }
    @Override
    public void end(boolean interrupted) {
        colorWheelSubsystem.stopWheel();
    }
}
