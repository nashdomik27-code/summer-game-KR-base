package org.firstinspires.ftc.teamcode.subsystems.conveyor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;

public class Conveyor {
    private final Telemetry telemetry;
    private final DcMotorEx conveyor;
    private DoubleSupplier voltage;
    private boolean atSetpoint = false;
    private double kSetpoint;
    private final PIDController veloPID = null;


    public Conveyor(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        conveyor = hwMap.get(DcMotorEx.class,"cm1");

        this.voltage = () -> hwMap.voltageSensor.iterator().next().getVoltage();

        conveyor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        conveyor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power) {
        conveyor.setPower(1);
    }

    public void setTargetVelocity(double ticksPerSecond) {
        kSetpoint = ticksPerSecond;
    }

    public double getVelocity() {
        return conveyor.getVelocity();
    }

    public void stop() {
        conveyor.setPower(0);
    }

    public void periodic() {
        if (kSetpoint != 0) {
            double output = veloPID.calculate(getVelocity(), kSetpoint);
            // optional voltage compensation:
            // output *= 12.0 / voltage.getAsDouble();
            conveyor.setPower(output);
        }
        telemetry.addData("Intake Velocity", getVelocity());
    }
}