// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/** An example command that uses an example subsystem. */
public class TelopDriveCommand extends CommandBase {
  private PIDController pidController;
  private double lastSpeed, lastTurn;
  private boolean armZeroed;
  private boolean flag;
  private Thread musicThread;
  /**
   * Creates a new TelopDriveCommand.
   */
  public TelopDriveCommand(PIDController pidController) {
    // Use addRequirements() here to declare subsystem dependencies.
    // this.addRequirements(RobotContainer.inst.tankDriveSubsystem);
    this.addRequirements(RobotContainer.inst.turretArmSubsystem);
    this.pidController = pidController;
    this.armZeroed = false;
    RobotContainer.inst.turretArmSubsystem.armMotor.setIdleMode(IdleMode.kCoast);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double speed = Constants.useAlternateControls ? RobotContainer.inst.controller.getLeftY() : RobotContainer.inst.controller.getRightTriggerAxis() - RobotContainer.inst.controller.getLeftTriggerAxis();
    // double turn = RobotContainer.inst.controller.getLeftX();
    // speed = MathUtil.applyDeadband(speed, 0.1);
    // turn = MathUtil.applyDeadband(turn, 0.1);
    // //speed = this.pidController.calculate(this.lastSpeed, speed);
    // //turn = this.pidController.calculate(this.lastTurn, turn);
    // this.lastSpeed = speed;
    // this.lastTurn = turn;
    // double rumbleDegreeMin = 4.0;
    // RobotContainer.inst.tankDriveSubsystem.leftSpeed = (speed + turn);
    // RobotContainer.inst.tankDriveSubsystem.rightSpeed = (speed - turn);
    
    // if (Math.abs(RobotContainer.inst.tankDriveSubsystem.navX.getRoll()) > rumbleDegreeMin) {
    //   RobotContainer.inst.controller.setRumble(RumbleType.kBothRumble, Math.abs(RobotContainer.inst.tankDriveSubsystem.navX.getRoll())/15.0);
    // } else if (Math.abs(RobotContainer.inst.tankDriveSubsystem.navX.getPitch()) > rumbleDegreeMin) {
    //   RobotContainer.inst.controller.setRumble(RumbleType.kBothRumble, Math.abs(RobotContainer.inst.tankDriveSubsystem.navX.getPitch())/15.0);
    // } else {
    //   RobotContainer.inst.controller.setRumble(RumbleType.kBothRumble, 0.0);
    // }

    if(RobotContainer.inst.controller.getLeftBumper() && !RobotContainer.inst.controller.getRightBumper()) {
      RobotContainer.inst.turretPivotSubsystem.turnCounterClockwise();
    } else if(RobotContainer.inst.controller.getRightBumper() && !RobotContainer.inst.controller.getLeftBumper()) {
      RobotContainer.inst.turretPivotSubsystem.turnClockwise();
    } else {
      RobotContainer.inst.turretPivotSubsystem.stopMotor();
    }

    // System.out.printf("Encoder & limit switch value: %f, %b\n", RobotContainer.inst.turretArmSubsystem.armEncoder.getDistance(), RobotContainer.inst.turretArmSubsystem.armLimSwitch.get());
    // if (!this.armZeroed) {
    //   if (!RobotContainer.inst.turretArmSubsystem.armLimSwitch.get()) {
    //     RobotContainer.inst.turretArmSubsystem.armMotor.stopMotor();
    //     RobotContainer.inst.turretArmSubsystem.armEncoder.reset();
    //     this.armZeroed = true;
    //   } else {
    //     RobotContainer.inst.turretArmSubsystem.armMotor.set(0.1);
    //   }
    // } else {
    //  if (RobotContainer.inst.controller.getLeftBumper() && !RobotContainer.inst.controller.getRightBumper() && RobotContainer.inst.turretArmSubsystem.armLimSwitch.get()) {
    //    RobotContainer.inst.turretArmSubsystem.armMotor.set(0.1);
    //  } else if (RobotContainer.inst.controller.getRightBumper() && !RobotContainer.inst.controller.getLeftBumper()) {
    //    RobotContainer.inst.turretArmSubsystem.armMotor.set(-0.1);
    //  } else {
    //    RobotContainer.inst.turretArmSubsystem.armMotor.set(0.0);
    //  }
    // }
  }

}
