package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motor.MotorGroup;

public class PassAndTossSubsystem extends SubsystemBase {

    private CANSparkMax tosser, flipper;
    private MotorGroup clawMotors;

    public PassAndTossSubsystem(CANSparkMax tosser, MotorGroup clawMotors, CANSparkMax flipper) {
        this.tosser = tosser;
        this.flipper = flipper;
        this.clawMotors = clawMotors;
    }

    public void startMacro() {
        
    }
    
}
