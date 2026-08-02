package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;

public class intake {

    private final Telemetry telemetry;
    private final DcMotorEx intake;
    private DoubleSupplier voltage;
    private double kSetpoint;
    private PIDController veloPID;


    public intake(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        intake = (hwMap.get(DcMotorEx.class, "shooter"));

        this.voltage = () -> hwMap.voltageSensor.iterator().next().getVoltage();

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        kSetpoint = ShooterConstants.setpoint;
        veloPID = new PIDController(ShooterConstants.kp, ShooterConstants.ki, ShooterConstants.kd);
        veloPID.setTolerance(10);
    }}
