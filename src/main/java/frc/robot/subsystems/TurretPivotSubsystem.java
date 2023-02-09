package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretPivotSubsystem extends SubsystemBase {
    public final CANSparkMax turretMotor;

    public TurretPivotSubsystem (CANSparkMax turretMotor) {
        this.turretMotor = turretMotor;
    }

    public void turnClockwise() {
        this.turretMotor.set(1.0);
    }

    public void turnCounterClockwise() {
        this.turretMotor.set(-1.0);
    }

    public void stopMotor() {
        this.turretMotor.stopMotor();
    }



}
