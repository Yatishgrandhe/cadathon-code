package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ColorWheelSubsystem;

public class RotateColorWheelCmd extends Command {
    private final ColorWheelSubsystem colorWheelSubsystem;

    public RotateColorWheelCmd(ColorWheelSubsystem colorWheelSubsystem) {
        this.colorWheelSubsystem = colorWheelSubsystem;
        addRequirements(colorWheelSubsystem);
    }

    @Override
    public void initialize() {
        colorWheelSubsystem.resetRotationCounter();
        colorWheelSubsystem.spinWheel(0.25);
    }
    @Override
    public void execute() {
    }
    @Override
    public boolean isFinished() {
        return colorWheelSubsystem.getRotations() >= 3.0;
    }
    @Override
    public void end(boolean interrupted) {
        colorWheelSubsystem.stopWheel();
    }
}
