package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathPoint;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    private final Follower drive;
    private final Telemetry telemetry;

    public Drive(HardwareMap hwMap, Telemetry telemetry) {
        drive = Constants.createFollower(hwMap);

//        drive.setPose(PoseStorage.currentPose);

        drive.followPath(new Path(new BezierPoint(new Pose())));
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        try {
            drive.update();
//            telemetry.addData("Drive T", drive.getCurrentTValue());
//            drive.telemetryDebug(telemetry);
        } catch (Exception ignored) {
        }
    }

    public void setPose(Pose pose) {
        drive.setPose(pose);
    }

    public void startTeleopDrive() {
        drive.startTeleopDrive();
    }

    public void drive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier) {
        drive.setTeleOpDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), rotationSupplier.getAsDouble(), false);
    }

    public Pose getPose() {
        return drive.getPose();
    }

    public PathPoint getDesiredPose() {
        return drive.getClosestPose();
    }

    public double getPathT() {
        return drive.getCurrentTValue();
    }

    public PathBuilder getPathBuilder() {
        return drive.pathBuilder();
    }

    public void followPath(PathChain path) {
        drive.followPath(path, true);
    }

    public boolean isFinished() {
        return !drive.isBusy();
    }

    public boolean headingIsFinished() {
        return drive.getHeadingError() < 0.001; // Adjust this threshold as needed
    }

    public static Command followPath(Drive drive, Pose startPose, PathChain path) {
        return Commands.runOnce(() -> drive.setPose(startPose), drive)
                .andThen(Commands.runOnce(() -> drive.followPath(path), drive));
    }

    public static Command followPath(Drive drive, PathChain path) {
        return Commands.runOnce(() -> drive.followPath(path), drive).andThen(Commands.waitUntil(drive::isFinished));
    }

    public static Command followPath(Drive drive, Supplier<PathChain> path) {
        return Commands.runOnce(() -> drive.followPath(path.get()), drive).andThen(Commands.waitUntil(drive::isFinished));
    }
}