package org.firstinspires.ftc.teamcode.subsystems.conveyor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.ftclib.hardware.motors.MotorEx;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {

    private final MotorEx conveyor;

    public Conveyor(HardwareMap hwMap) {

        conveyor = new MotorEx(hwMap, "cm1");

    }


    public void setPower(double power) {
        conveyor.set(power);
    }


    public static Command setPower(Conveyor conveyor, DoubleSupplier power) {
        return Commands.run(() -> conveyor.setPower(power.getAsDouble()), conveyor);
    }

    public static Command setPower(Conveyor conveyor, double power) {
        return setPower(conveyor, () -> power);
    }
}