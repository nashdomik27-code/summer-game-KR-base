package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterConstants {
    public static double kp=0,ki=0,kd=0,kv=0;
    public static double setpoint = 0;
    public static double midShot = 1020;
    public static double longShot = 1350;
    public static double idle = 0;
}
