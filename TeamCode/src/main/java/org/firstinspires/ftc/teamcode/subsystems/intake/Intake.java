package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.ftclib.hardware.motors.MotorEx;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final MotorEx intake;

    public Intake(HardwareMap hwMap) {

        intake = new MotorEx(hwMap, "cm0");


    }


    public void setPower(double power) {
        intake.set(power);
    }


    public static Command setPower(Intake intake, DoubleSupplier power) {
        return Commands.run(() -> intake.setPower(power.getAsDouble()), intake);
    }

    public static Command setPower(Intake intake, double power) {
        return setPower(intake, () -> power);
    }
}