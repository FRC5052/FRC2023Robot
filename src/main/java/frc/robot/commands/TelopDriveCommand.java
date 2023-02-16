// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import static frc.robot.RobotContainer.robot;

/** An example command that uses an example subsystem. */
public class TelopDriveCommand extends CommandBase {
  private PIDController pidController;
  private double lastSpeed, lastTurn;
  private boolean armZeroed;
  /**
   * Creates a new TelopDriveCommand.
   */
  public TelopDriveCommand(PIDController pidController) {
    // Use addRequirements() here to declare subsystem dependencies.
    if (robot.tankDriveSubsystem != null) this.addRequirements(robot.tankDriveSubsystem);
    if (robot.turretArmSubsystem != null) this.addRequirements(robot.turretArmSubsystem);
    if (robot.turretPivotSubsystem != null) this.addRequirements(robot.turretPivotSubsystem);
    this.pidController = pidController;
    this.armZeroed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Tank Drive control code
    if (robot.tankDriveSubsystem != null) {
      double speed = Constants.useAlternateControls ? robot.controller.getLeftY() : robot.controller.getRightTriggerAxis() - robot.controller.getLeftTriggerAxis();
      double turn = robot.controller.getLeftX();
      speed = MathUtil.applyDeadband(speed, 0.1);
      turn = MathUtil.applyDeadband(turn, 0.1);
      // speed = this.pidController.calculate(this.lastSpeed, speed);
      // turn = this.pidController.calculate(this.lastTurn, turn);
      this.lastSpeed = speed;
      this.lastTurn = turn;
      double rumbleDegreeMin = 4.0;
      robot.tankDriveSubsystem.leftSpeed = (speed + turn);
      robot.tankDriveSubsystem.rightSpeed = (speed - turn);
    
      if (Math.abs(robot.tankDriveSubsystem.navX.getRoll()) > rumbleDegreeMin) {
        robot.controller.setRumble(RumbleType.kBothRumble, Math.abs(robot.tankDriveSubsystem.navX.getRoll())/15.0);
      } else if (Math.abs(robot.tankDriveSubsystem.navX.getPitch()) > rumbleDegreeMin) {
        robot.controller.setRumble(RumbleType.kBothRumble, Math.abs(robot.tankDriveSubsystem.navX.getPitch())/15.0);
      } else {
        robot.controller.setRumble(RumbleType.kBothRumble, 0.0);
      }
    }

    // Turret Pivot control code
    if (robot.turretPivotSubsystem != null) {
      if(robot.controller.getLeftBumper() && !robot.controller.getRightBumper()) {
        robot.turretPivotSubsystem.turnCounterClockwise();
      } else if(robot.controller.getRightBumper() && !robot.controller.getLeftBumper()) {
        robot.turretPivotSubsystem.turnClockwise();
      } else {
        robot.turretPivotSubsystem.stopMotor();
      }
    }
    
    // Turret Arm control code
    if (robot.turretArmSubsystem != null) {
      System.out.printf("Encoder & limit switch value: %f, %b\n", robot.turretArmSubsystem.armEncoder.getDistance(), robot.turretArmSubsystem.armLimSwitch.get());
      if (!this.armZeroed) {
        if (!robot.turretArmSubsystem.armLimSwitch.get()) {
          robot.turretArmSubsystem.armMotor.stopMotor();
          robot.turretArmSubsystem.armEncoder.reset();
          this.armZeroed = true;
        } else {
          robot.turretArmSubsystem.armMotor.set(0.1);
        }
      } else {
        if (robot.controller.getLeftBumper() && !robot.controller.getRightBumper() && robot.turretArmSubsystem.armLimSwitch.get()) {
          robot.turretArmSubsystem.armMotor.set(0.1);
        } else if (robot.controller.getRightBumper() && !robot.controller.getLeftBumper()) {
          robot.turretArmSubsystem.armMotor.set(-0.1);
        } else {
          robot.turretArmSubsystem.armMotor.stopMotor();
        }
      }
    }
  }

}
