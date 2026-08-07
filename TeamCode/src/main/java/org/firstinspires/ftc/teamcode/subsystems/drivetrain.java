package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class drivetrain {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    private double previousX = 0;
    private double previousY = 0;
    private double previousTurn = 0;

    private static final double ACCEL_LIMIT = 1;

    public drivetrain(HardwareMap hardwareMap) {

        frontLeft  = hardwareMap.get(DcMotorEx.class, "cm0");
        frontRight = hardwareMap.get(DcMotorEx.class, "cm1");
        backLeft   = hardwareMap.get(DcMotorEx.class, "cm2");
        backRight  = hardwareMap.get(DcMotorEx.class, "cm3");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);  
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(parameters);
    }

    public void drive(double y, double x, double rx, boolean fieldCentric, boolean slowMode) {

        // Cubic joystick response
        y = Math.pow(y,3);
        x = Math.pow(x,3);
        rx = Math.pow(rx,3);

        // Acceleration limiting
        y = ramp(previousY, y);
        x = ramp(previousX, x);
        rx = ramp(previousTurn, rx);

        previousY = y;
        previousX = x;
        previousTurn = rx;

        if(fieldCentric){

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

            x = rotX;
            y = rotY;
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx),1);

        double fl = (y + x + rx)/denominator;
        double bl = (y - x + rx)/denominator;
        double fr = (y - x - rx)/denominator;
        double br = (y + x - rx)/denominator;

        if(slowMode){
            fl *= 0.35;
            fr *= 0.35;
            bl *= 0.35;
            br *= 0.35;
        }

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private double ramp(double previous, double current){

        double delta = current - previous;

        delta = Range.clip(delta,-ACCEL_LIMIT,ACCEL_LIMIT);

        return previous + delta;
    }

    public void resetHeading(){
        imu.resetYaw();
    }

}