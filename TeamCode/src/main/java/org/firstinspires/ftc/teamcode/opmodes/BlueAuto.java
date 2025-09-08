package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.lib.ftclib.opmode.CommandOpMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BlueAuto", group = "Auto")
public class BlueAuto extends CommandOpMode {
    private RobotContainer robotContainer;
    private Telemetry robotTelemetry;
    private Timer timer = new Timer();

    private double previousTime;

    @Override
    public void robotInit() {
        robotTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robotContainer = new RobotContainer(hardwareMap, telemetry, gamepad1, gamepad2, OpModeConstants.BLUE_AUTO); //Uses heavily modified untested hardware
        timer.start();
    }

    @Override
    public void robotPeriodic() {
        super.robotPeriodic();
        robotTelemetry.addData("Loop Time", 1.0 / (timer.get() - previousTime));
        robotTelemetry.update();

        previousTime = timer.get();
    }

    @Override
    public void enabledInit() {
        robotContainer.getAutoCommand(OpModeConstants.BLUE_AUTO).schedule();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().reset();
    }
}