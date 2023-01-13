// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TelopDriveCommand extends CommandBase {
  /**
   * Creates a new TelopDriveCommand.
   */
  public TelopDriveCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(RobotContainer.inst.tankDriveSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = Constants.useAlternateControls ? RobotContainer.inst.controller.getLeftY() : RobotContainer.inst.controller.getRightTriggerAxis() - RobotContainer.inst.controller.getLeftTriggerAxis();
    double turn = RobotContainer.inst.controller.getLeftX();
    RobotContainer.inst.tankDriveSubsystem.leftSpeed = (speed + turn) / 2.0;
    RobotContainer.inst.tankDriveSubsystem.rightSpeed = (speed - turn) / 2.0;
  }
}
