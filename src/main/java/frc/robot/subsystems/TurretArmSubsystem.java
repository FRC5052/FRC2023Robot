// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretArmSubsystem extends SubsystemBase {
  public final CANSparkMax armMotor;
  public final DutyCycleEncoder armEncoder;
  public final DigitalInput armLimSwitch;
  public static final double maxEncoderAngle = 20.0;

  /** Creates a new ExampleSubsystem. */
  public TurretArmSubsystem(CANSparkMax armMotor, DutyCycleEncoder armEncoder, DigitalInput armLimSwitch) {
    this.armMotor = armMotor;
    this.armEncoder = armEncoder;
    this.armLimSwitch = armLimSwitch;
    this.armEncoder.reset();
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
