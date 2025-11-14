package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TheForce_Hanging;

public class HangingCmd extends Command {
    private final TheForce_Hanging hangingSubsystem;
    
    private final Supplier<Boolean> upButtonFunction;
    private final Supplier<Boolean> downButtonFunction;
    private final Supplier<Boolean> leftButtonFunction;
    private final Supplier<Boolean> rightButtonFunction;
    private final Supplier<Boolean> hookButtonFunction;
    private final Supplier<Boolean> unhookButtonFunction;
    
    private static final double VERTICAL_UP_VOLTS = 3.0;
    private static final double VERTICAL_DOWN_VOLTS = -3.0;

    public HangingCmd(TheForce_Hanging hangingSubsystem,
                      Supplier<Boolean> upButtonFunction,
                      Supplier<Boolean> downButtonFunction,
                      Supplier<Boolean> leftButtonFunction,
                      Supplier<Boolean> rightButtonFunction,
                      Supplier<Boolean> hookButtonFunction,
                      Supplier<Boolean> unhookButtonFunction) {
        this.hangingSubsystem = hangingSubsystem;
        this.upButtonFunction = upButtonFunction;
        this.downButtonFunction = downButtonFunction;
        this.leftButtonFunction = leftButtonFunction;
        this.rightButtonFunction = rightButtonFunction;
        this.hookButtonFunction = hookButtonFunction;
        this.unhookButtonFunction = unhookButtonFunction;
        addRequirements(hangingSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        boolean upButton = upButtonFunction.get();
        boolean downButton = downButtonFunction.get();
        boolean leftButton = leftButtonFunction.get();
        boolean rightButton = rightButtonFunction.get();
        boolean hookButton = hookButtonFunction.get();
        boolean unhookButton = unhookButtonFunction.get();
        
        double verticalPower = 0.0;
        if (upButton && !downButton) {
            verticalPower = VERTICAL_UP_VOLTS;
        } else if (downButton && !upButton) {
            verticalPower = VERTICAL_DOWN_VOLTS;
        }
        hangingSubsystem.setVerticalMotorPower(verticalPower);
        
        double sidewaysPower = 0.0;
        if (rightButton && !leftButton) {
            sidewaysPower = TheForce_Hanging.SIDEWAYS_VOLTS;
        } else if (leftButton && !rightButton) {
            sidewaysPower = -TheForce_Hanging.SIDEWAYS_VOLTS;
        }
        hangingSubsystem.setSidewaysMotorPower(sidewaysPower);
        
        if (hookButton && !unhookButton) {
            hangingSubsystem.hook();
        } else if (unhookButton && !hookButton) {
            hangingSubsystem.unhook();
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        hangingSubsystem.setVerticalMotorPower(0.0);
        hangingSubsystem.setSidewaysMotorPower(0.0);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
