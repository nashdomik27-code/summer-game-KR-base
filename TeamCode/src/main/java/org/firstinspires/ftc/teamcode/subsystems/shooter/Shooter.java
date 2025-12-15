package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    //region Constants
    private final Telemetry telemetry;

    private final DcMotorEx shooter;
    private final DcMotorEx shooter2;

    private double kSetpoint;

    private double currentVelo = 0;
    private double desiredVelo = 0;
    private PIDController veloPID;

    private boolean atSetpoint = false;

    private DoubleSupplier voltage;
    //endregion

    public Shooter(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        shooter = (hwMap.get(DcMotorEx.class, "shooter"));
        shooter2 = (hwMap.get(DcMotorEx.class, "shooter2"));

        this.voltage = () -> hwMap.voltageSensor.iterator().next().getVoltage();

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        kSetpoint = ShooterConstants.setpoint;
        veloPID = new PIDController(ShooterConstants.kp, ShooterConstants.ki, ShooterConstants.kd);
        veloPID.setTolerance(10);
    }

    @Override
    public void periodic() {
        try {
            veloPID.setPID(ShooterConstants.kp, ShooterConstants.ki, ShooterConstants.kd);

            if (kSetpoint != ShooterConstants.setpoint) {
                kSetpoint = ShooterConstants.setpoint;
                desiredVelo = kSetpoint;
            }
            currentVelo = (shooter.getVelocity());

            double pwr = veloPID.calculate(currentVelo - desiredVelo);
            double ff = ShooterConstants.kv * desiredVelo;
            double output = pwr + ff;

            shooter.setPower((output/voltage.getAsDouble()*12));
            shooter2.setPower((output/voltage.getAsDouble()*12));

            telemetry.addData("Target Velocity", desiredVelo);
            telemetry.addData("Current Velocity", currentVelo);
            telemetry.addData("Motor 1 AMPS", shooter.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Motor 2 AMPS", shooter2.getCurrent(CurrentUnit.AMPS));

        } catch (Exception ignored) {

        }
    }

    public BooleanSupplier atSetpoint() {
        if (desiredVelo == 0) {
            atSetpoint = false;
            return () -> atSetpoint;
        }

        atSetpoint = Math.abs(currentVelo) >= 0.9 * Math.abs(desiredVelo);
        return () -> atSetpoint;
    }

    //region Commands
    public void setPower(double power) {
        shooter.setPower(power);
    }

    public static Command setPower(Shooter shooter, DoubleSupplier power) {
        return Commands.run(
                () -> shooter.setPower(power.getAsDouble()), shooter
        );
    }

    private void setVelocity(double rpm) {
        desiredVelo = rpm;
    }

    public static Command setVelocity(Shooter shooter, DoubleSupplier rpm) {
        return Commands.run(
                () -> shooter.setVelocity(rpm.getAsDouble()), shooter
        );
    }
    //endregion

}