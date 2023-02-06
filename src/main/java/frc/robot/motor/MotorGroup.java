package frc.robot.motor;

import java.util.Arrays;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class MotorGroup {
    public final CANSparkMax leader;
    public final CANSparkMax[] followers;

    public MotorGroup(CANSparkMax leader, CANSparkMax... followers) {
        this.leader = leader;
        this.followers = followers;
        for (int i = 0; i < this.followers.length; i++) {
            this.followers[i].follow(this.leader);
        }

    }

    public MotorGroup(int leaderID, int... followerIDs) {
        this.leader = new CANSparkMax(leaderID, MotorType.kBrushless);
        this.followers = new CANSparkMax[followerIDs.length];
        for (int i = 0; i < this.followers.length; i++) {
            this.followers[i] = new CANSparkMax(followerIDs[i], MotorType.kBrushless);
            this.followers[i].follow(this.leader);
        }
    }

    public void setInverted(boolean inverted) {
        this.leader.setInverted(inverted);
        for (CANSparkMax follower : this.followers) {
            follower.setInverted(inverted);
        }
    }

    public void setIdleMode(IdleMode mode) {
        this.leader.setIdleMode(mode);
        for (CANSparkMax follower : this.followers) {
            follower.setIdleMode(mode);
        }
    }

    public void set(double value) {
        this.leader.set(value);
    }

    public double get() {
        return this.leader.get();
    }
}
