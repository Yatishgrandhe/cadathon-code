package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TheForce_Hanging;

// This command extends Command to control the hanging mechanism with buttons
public class HangingCmd extends Command {
    // Subsystem is stored to access hanging mechanism functionality
    private final TheForce_Hanging hangingSubsystem;
    // Boolean suppliers allow button states to be read fresh each execute() call
    private final Supplier<Boolean> upButtonFunction;
    private final Supplier<Boolean> downButtonFunction;
    
    // Voltage constants for up and down movement
    private static final double UP_VOLTS = 3.0;
    private static final double DOWN_VOLTS = -3.0;

    // Constructor takes suppliers as parameters to decouple this command from specific input devices
    public HangingCmd(TheForce_Hanging hangingSubsystem,
                      Supplier<Boolean> upButtonFunction,
                      Supplier<Boolean> downButtonFunction) {
        this.hangingSubsystem = hangingSubsystem;
        this.upButtonFunction = upButtonFunction;
        this.downButtonFunction = downButtonFunction;
        // addRequirements prevents other commands from using the hanging subsystem simultaneously
        addRequirements(hangingSubsystem);
    }

    @Override
    // Empty because no initialization needed - command runs continuously
    public void initialize() {
    }

    @Override
    // Execute runs every robot cycle to continuously read button input and control the hanging mechanism
    public void execute() {
        // Read button states fresh each cycle to get real-time input
        boolean upButton = upButtonFunction.get();
        boolean downButton = downButtonFunction.get();
        
        // Determine motor power based on button states
        double motorPower = 0.0;
        if (upButton && !downButton) {
            // Up button pressed (and down not pressed) - move up
            motorPower = UP_VOLTS;
        } else if (downButton && !upButton) {
            // Down button pressed (and up not pressed) - move down
            motorPower = DOWN_VOLTS;
        }
        // If both buttons pressed or neither pressed, motorPower stays 0.0
        
        // Set motor power (you'll need to make setMotorPower public in TheForce_Hanging)
        hangingSubsystem.setMotorPower(motorPower);

        }


}