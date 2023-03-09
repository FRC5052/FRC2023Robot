// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import static frc.robot.RobotContainer.robot;


/** An example command that uses an example subsystem. */
public class TelopDriveCommand extends CommandBase {
  private static double rumbleDegreeMin = 4.0;
  private PIDController pidController;
  private double lastSpeed, lastTurn;
  private boolean armZeroed;
  private boolean tankDriveToggle;
  private boolean brakeToggle;
  /**
   * Creates a new TelopDriveCommand.
   */
  public TelopDriveCommand(PIDController pidController) {
    // Use addRequirements() here to declare subsystem dependencies.
    if (robot.tankDriveSubsystem != null) this.addRequirements(robot.tankDriveSubsystem);
    if (robot.turretArmSubsystem != null) this.addRequirements(robot.turretArmSubsystem);
    if (robot.turretPivotSubsystem != null) this.addRequirements(robot.turretPivotSubsystem);
    this.pidController = pidController;
    this.armZeroed = true;
    this.tankDriveToggle = false;
    this.brakeToggle = false;

    CameraServer.startAutomaticCapture();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Tank Drive control code
    if (robot.tankDriveSubsystem != null) {
      robot.tankDriveSubsystem.setIdleMode(null);
      // Checks the back/options button on the drive controller and if it is newly pressed this cycle, toggle between tank drive and rocket league/arcade drive
      if (robot.driveController.getBackButtonPressed()) this.tankDriveToggle = !this.tankDriveToggle;
      // Detects if in tank drive mode
      if (this.tankDriveToggle) { // Tank drive
        // Read controller inputs
        double left = robot.driveController.getLeftY() * 0.8;
        double right = robot.driveController.getRightY() * 0.8;
        
        // Deadband 
        left = MathUtil.applyDeadband(-left, 0.1);
        right = MathUtil.applyDeadband(-right, 0.1);

        // Tell subsystem to set the motor speeds to the given values
        robot.tankDriveSubsystem.setSpeeds(left, right);
      } else { // Rocket League / Arcade drive
        // Read controller 
        double speed = Constants.useAlternateControls ? robot.driveController.getLeftY() : robot.driveController.getRightTriggerAxis() - robot.driveController.getLeftTriggerAxis();
        double turn = robot.driveController.getLeftX();
        speed = MathUtil.applyDeadband(speed, 0.1);
        turn = MathUtil.applyDeadband(turn, 0.1);
        robot.tankDriveSubsystem.setSpeeds((speed + turn), (speed - turn));
      }
      System.out.printf("Pitch: %f\n", robot.tankDriveSubsystem.navX.getPitch());
      if (Math.abs(robot.tankDriveSubsystem.navX.getRoll()) > rumbleDegreeMin) {
        robot.driveController.setRumble(RumbleType.kBothRumble, Math.abs(robot.tankDriveSubsystem.navX.getRoll())/15.0);
      } else if (Math.abs(robot.tankDriveSubsystem.navX.getPitch()) > rumbleDegreeMin) {
        robot.driveController.setRumble(RumbleType.kBothRumble, Math.abs(robot.tankDriveSubsystem.navX.getPitch())/15.0);
      } else {
        robot.driveController.setRumble(RumbleType.kBothRumble, 0.0);
      }
    }

    // Turret Pivot control code
    if (robot.turretPivotSubsystem != null) {
      robot.turretPivotSubsystem.turn(MathUtil.applyDeadband(robot.turretController.getLeftX()*0.75, 0.1));
    }
    
    // Turret Arm control code
    if (robot.turretArmSubsystem != null) {
      // if (!this.armZeroed) {
      //   if (robot.turretArmSubsystem.limitSwitchPressed()) {
      //     robot.turretArmSubsystem.lock();
      //     robot.turretArmSubsystem.resetEncoder();
      //     this.armZeroed = true;
      //   } else {
      //     robot.turretArmSubsystem.unlock();
      //   }
      // } else {
      //   if (robot.turretController.getLeftBumper() && !robot.turretController.getRightBumper()) {
      //     robot.turretArmSubsystem.moveDown();
      //   } else if (robot.turretController.getRightBumper() && !robot.turretController.getLeftBumper()) {
      //     robot.turretArmSubsystem.moveUp();
      //   } else {
      //     robot.turretArmSubsystem.stopMoving();
      //   }
      // }
      robot.turretArmSubsystem.move(MathUtil.applyDeadband(robot.turretController.getLeftY()*0.75, 0.1));
    }

    if (robot.turretClawSubsystem != null) {
      if (robot.turretController.getLeftBumper()) {
        robot.turretClawSubsystem.closeClaw();
      } else if (robot.turretController.getRightBumper()) {
        robot.turretClawSubsystem.openClaw();
      } else {
        robot.turretClawSubsystem.stopClaw();
      }
    }
  }
}
