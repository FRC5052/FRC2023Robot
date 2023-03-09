// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutonCommand;
import frc.robot.commands.SideAutonCommand;
import frc.robot.commands.TelopDriveCommand;
import frc.robot.motor.MotorGroup;
import frc.robot.subsystems.TankDriveSubsystem;
import frc.robot.subsystems.TurretArmSubsystem;
import frc.robot.subsystems.TurretClawSubsystem;
import frc.robot.subsystems.TurretPivotSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The primary instance of the robot (technically the last one initialized).
  public static RobotContainer robot = null;

  public final XboxController driveController;
  public final XboxController turretController;

  public final TelopDriveCommand telopDriveCommand;
  public final AutonCommand autonCommand;
  public final SideAutonCommand sideAutonCommand;

  public final TankDriveSubsystem tankDriveSubsystem;
  public final TurretArmSubsystem turretArmSubsystem;
  public final TurretPivotSubsystem turretPivotSubsystem;
  public final TurretClawSubsystem turretClawSubsystem;

  public static RobotContainer getRobot() {
    return RobotContainer.robot;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (RobotContainer.robot == null) RobotContainer.robot = this;
    this.driveController = new XboxController(0);
    this.turretController = new XboxController(1);

    // this.tankDriveSubsystem = null;
    this.tankDriveSubsystem = new TankDriveSubsystem(
      new MotorGroup(Constants.leftLeaderID, Constants.leftFollowerID), 
      new MotorGroup(Constants.rightLeaderID, Constants.rightFollowerID), 
      new PIDController(0.6, 0.5, 0.0), // Output PID
      new AHRS()
    );

    this.turretArmSubsystem = null;
    // this.turretArmSubsystem = new TurretArmSubsystem(
    //   new CANSparkMax(Constants.armMotorID, MotorType.kBrushless), //new CANSparkMax(Constants.armMotorID, MotorType.kBrushless),
    //   new DigitalInput(Constants.armLimSwitchPort)
    // );

    this.turretPivotSubsystem = null;
    // this.turretPivotSubsystem = new TurretPivotSubsystem(
    //   new CANSparkMax(Constants.pivotMotorID, MotorType.kBrushless)
    // );

    this.turretClawSubsystem = null;
    // this.turretClawSubsystem = new TurretClawSubsystem(
    //   new MotorGroup(Constants.clawLeaderID, Constants.clawFollowerID)
    // );

    this.telopDriveCommand = new TelopDriveCommand(
      new PIDController(0.6, 0.5, 0.0) // Input PID
    );

    this.autonCommand = new AutonCommand();
    this.sideAutonCommand = new SideAutonCommand();


    // Configure the button bindings
    this.configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonCommand;
  }

  public Command getTeleopCommand() {
    return this.telopDriveCommand;
  }
}
