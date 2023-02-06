// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretArmSubsystem extends SubsystemBase {
  public final CANSparkMax armMotor;
  public final Encoder armEncoder;
  public final DigitalInput armLimSwitch;

  /** Creates a new ExampleSubsystem. */
  public TurretArmSubsystem(CANSparkMax armMotor, Encoder armEncoder, DigitalInput armLimSwitch) {
    this.armMotor = armMotor;
    this.armEncoder = armEncoder;
    this.armLimSwitch = armLimSwitch;
    this.armEncoder.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
