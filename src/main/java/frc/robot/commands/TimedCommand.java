package frc.robot.commands;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandBase;

public abstract class TimedCommand extends CommandBase {
    protected long ticks;

    protected double getSeconds() {
        return ((double)this.ticks)*TimedRobot.kDefaultPeriod;
    }

    @Override
    public void initialize() {
        this.ticks = 0;
    }

    @Override
    public void execute() {
        this.ticks++;
    }
}
