package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.lib.ftclib.opmode.CommandOpMode;

import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends CommandOpMode {
    private Telemetry robotTelemetry;
    private Timer timer = new Timer();

    private double previousTime;

    List<LynxModule> allHubs;

    @Override
    public void robotInit() {
        robotTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        new RobotContainer(hardwareMap, robotTelemetry, gamepad1, gamepad2, OpModeConstants.TELEOP); //Uses heavily modified untested hardware
        timer.start();

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void robotPeriodic() {
        super.robotPeriodic();
        robotTelemetry.addData("Loop Time", 1.0 / (timer.get() - previousTime));
        robotTelemetry.update();

        previousTime = timer.get();

        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }
}