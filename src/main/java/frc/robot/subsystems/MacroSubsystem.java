package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.PassAndTossCommand;

public class MacroSubsystem extends SubsystemBase {
    PassAndTossCommand passAndTossCommand;

    public MacroSubsystem() {
        passAndTossCommand = new PassAndTossCommand();
    }

    @Override
    public void periodic(){
        passAndTossCommand.execute();
    }
}
