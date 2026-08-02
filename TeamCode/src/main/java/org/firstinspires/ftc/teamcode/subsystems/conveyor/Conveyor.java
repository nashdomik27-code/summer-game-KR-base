package org.firstinspires.ftc.teamcode.subsystems.conveyor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.function.DoubleSupplier;

public class Conveyor {
    private final Telemetry telemetry;
    private final DcMotorEx conveyor;
    private DoubleSupplier voltage;
    private boolean atSetpoint = false;

    public Conveyor(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        conveyor = hwMap.get(DcMotorEx.class, "cm1");

        this.voltage = () -> hwMap.voltageSensor.iterator().next().getVoltage();

        conveyor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        conveyor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}