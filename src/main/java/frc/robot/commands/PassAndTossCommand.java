// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.RobotContainer.robot;

/** An example command that uses an example subsystem. */
public class PassAndTossCommand extends CommandBase {
  private enum PATState {
    kWaitClawUp,
    kWaitClawOpen,
    kWaitClawDown,
    kWaitTosserUp,
    kWaitTosserDown,
    kEnd,
    ;
  }

  private PATState state = PATState.kWaitClawUp;

  public PassAndTossCommand() {
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.state = PATState.kWaitClawUp;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (this.state) {
      case kWaitClawUp:
        {
          robot.turretClawSubsystem.setDumping();
          if (robot.turretClawSubsystem.clawFlipAtDesiredPos()) this.state = PATState.kWaitClawOpen;
        }
        break;
      case kWaitClawOpen:
        {
          robot.turretClawSubsystem.openClaw();
          if (robot.turretClawSubsystem.clawIsAtDesiredPos()) this.state = PATState.kWaitClawDown;
        }
        break;
      case kWaitClawDown:
        {
          robot.turretClawSubsystem.setIntake();
          if (robot.turretClawSubsystem.clawFlipAtDesiredPos()) this.state = PATState.kWaitTosserUp;
        }
        break;
      case kWaitTosserUp:
        {
          robot.shooterSubsystem.feed();
          if (robot.shooterSubsystem.isAtDesiredPosition()) this.state = PATState.kWaitTosserDown;
        }
        break;
      case kWaitTosserDown:
        {
          robot.shooterSubsystem.down();
          if (robot.shooterSubsystem.isAtDesiredPosition()) this.state = PATState.kEnd;
        }
        break;
      default:
        break;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.state == PATState.kEnd;
  }
}
