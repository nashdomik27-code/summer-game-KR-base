package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intake extends SubsystemBase {

    private final Telemetry telemetry;
    private final DcMotorEx intake;
    private final DoubleSupplier voltage;
    private double kSetpoint;
    private final PIDController veloPID;

    public intake(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        intake = hwMap.get(DcMotorEx.class, "cm2");
        this.voltage = () -> hwMap.voltageSensor.iterator().next().getVoltage();

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        veloPID = new PIDController(0.0, 0.0, 0.0); // tune these
    }

    public void setPower(double power) {
        intake.setPower(1);
    }

    public void setTargetVelocity(double ticksPerSecond) {
        kSetpoint = ticksPerSecond;
    }

    public double getVelocity() {
        return intake.getVelocity();
    }

    public void stop() {
        intake.setPower(0);
    }

    @Override
    public void periodic() {
        if (kSetpoint != 0) {
            double output = veloPID.calculate(getVelocity(), kSetpoint);
            // optional voltage compensation:
            // output *= 12.0 / voltage.getAsDouble();
            intake.setPower(output);
        }
        telemetry.addData("Intake Velocity", getVelocity());
    }
}