package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase{

    public double x;
    public double y;
    public double area;

    public NetworkTable table;
    public NetworkTableEntry tx;
    public NetworkTableEntry ty;
    public NetworkTableEntry ta;
    
    public LimelightSubsystem() {
        this.table = NetworkTableInstance.getDefault().getTable("limelight");
        this.tx = table.getEntry("tx");
        this.ty = table.getEntry("ty");
        this.ta = table.getEntry("ta");
    }

    @Override
    public void periodic() {
        // //read values periodically
        // x = tx.getDouble(0.0);
        // y = ty.getDouble(0.0);
        // area = ta.getDouble(0.0);
              
        // //post to smart dashboard periodically
        // SmartDashboard.putNumber("LimelightX", x);
        // SmartDashboard.putNumber("LimelightY", y);
        // SmartDashboard.putNumber("LimelightArea", area);

        // System.out.printf("LimelightX: %f \n", x);
        // System.out.printf("LimelightY: %f \n", y);
        // System.out.printf("LimelightArea: %f \n", area);
    }
}
