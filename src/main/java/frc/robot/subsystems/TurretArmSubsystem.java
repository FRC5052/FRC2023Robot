// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import static frc.robot.RobotContainer.robot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretArmSubsystem extends SubsystemBase {
  public final CANSparkMax armMotor;
  public final RelativeEncoder armEncoder;
  public final DigitalInput armLimSwitch;
  public static final double maxEncoderAngle = 20.0;

  /** Creates a new ExampleSubsystem. */
  public TurretArmSubsystem(CANSparkMax armMotor, DigitalInput armLimSwitch) {
    this.armMotor = armMotor;
    this.armEncoder = armMotor.getEncoder();
    this.armLimSwitch = armLimSwitch;
  }

  // Make the arm return to home under its own weight
  public void unlock() {
    this.armMotor.setIdleMode(IdleMode.kBrake);
  }

  // Make the arm resist gravity
  public void lock() {
    this.armMotor.setIdleMode(IdleMode.kBrake);
  }

  public void moveDown() {
    this.armMotor.set(0.1);
  }

  public void moveUp() {
    this.armMotor.set(-0.1);
  }

  public void stopMoving() {
    this.armMotor.stopMotor();
  }

  public void resetEncoder() {
    this.armEncoder.setPosition(0.0);
  }

  public boolean limitSwitchPressed() {
    return !this.armLimSwitch.get();
  }

  // // angle is 0.0 for home (contracted) angle, 1.0 is max legal angle (extended)
  // public void gotoAngle(double angle) {
    
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
