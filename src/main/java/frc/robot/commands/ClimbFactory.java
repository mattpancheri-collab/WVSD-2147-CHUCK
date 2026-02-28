package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public final class ClimbFactory {
    private ClimbFactory() {
    }

    public static Command climbCommand(Climber climber) {
        return climber.climbCommand();
    }

    public static Command reverseClimbCommand(Climber climber) {
        return climber.reverseClimbCommand();
    }
}
