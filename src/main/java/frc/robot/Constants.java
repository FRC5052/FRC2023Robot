// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // false for Rocket League controls, true for Arcade controls
    public static final boolean useAlternateControls = false;

    public static final int leftLeaderID = 2;
    public static final int leftFollowerID = 1;
    public static final int rightLeaderID = 3;
    public static final int rightFollowerID = 4;

    public static final int armMotorID = 5;
    public static final int[] armEncoderPorts = {0, 1, 2, 3};
    public static final int armLimSwitchPort = 4;

    public static final int pivotMotorId = 6;


// stuff below is constants for Auton dont touch unless u know what ur doing 

    // once the robot is put together we can calculate the info using SysID before and plug it in 
    public static final double ksVolts = 0;
    public static final double KvVoltSecondsPerMeter = 0;
    public static final double kaVoltSecondsSquaredPerMeter = 0;
    public static final double kPDriveVel = 0;


}
