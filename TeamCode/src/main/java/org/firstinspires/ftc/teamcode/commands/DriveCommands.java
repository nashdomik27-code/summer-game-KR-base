package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;

import org.firstinspires.ftc.teamcode.commands.auto.PoseStorage;
import org.firstinspires.ftc.teamcode.lib.controller.SquIDController;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class DriveCommands {

    private DriveCommands() {}

    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     */
    public static Command joystickDriveAtAngle(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotationSupplier) {

        // Create PID controller
        SquIDController angleController =
                new SquIDController(
                        0.1);

        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return Commands.runOnce(drive::startTeleopDrive, drive).andThen(
                Commands.run(
                        () -> {
                            // Calculate angular speed
                            double omega =
                                    angleController.calculate(
                                            drive.getPose().getHeading(), rotationSupplier.getAsDouble());

                            drive.drive(
                                    xSupplier,
                                    ySupplier,
                                    () -> omega);
                        },
                        drive));
    }

    public static Command driveToPose(Drive drive, Supplier<Pose> pose) {
        return Drive.followPath(
                drive,
                () -> drive.getPathBuilder()
                        .addPath(new Path(new BezierLine(drive.getPose(), pose.get())))
                        .setLinearHeadingInterpolation(drive.getPose().getHeading(), pose.get().getHeading())
                        .build());
    }

    public static Command driveToPose(Drive drive, Supplier<Pose> pose, Command command, DoubleSupplier commandActivationPoint) {
        return Commands.parallel(
                driveToPose(drive, pose),
                Commands.sequence(
                        Commands.waitUntil(() -> drive.getPathT() > commandActivationPoint.getAsDouble()),
                        command
                )
        );
    }

    public static Command setPose(Drive drive, Supplier<Pose> pose) {
        return Commands.runOnce(() -> drive.setPose(pose.get()), drive);
    }

    public static Command forward(Drive drive, DoubleSupplier distance) {
        return Drive.followPath(
                drive,
                () -> drive.getPathBuilder()
                        .addPath(new Path(new BezierLine(
                                drive.getDesiredPose().getPose(),
                                new Pose(
                                        drive.getDesiredPose().getPose().getX() + Math.cos(drive.getPose().getHeading()) * distance.getAsDouble(),
                                        drive.getDesiredPose().getPose().getY() + Math.sin(drive.getPose().getHeading()) * distance.getAsDouble(),
                                        drive.getDesiredPose().getPose().getHeading()))))
                        .setLinearHeadingInterpolation(drive.getPose().getHeading(), drive.getPose().getHeading())
                        .build());
    }

    public static Command backward(Drive drive, DoubleSupplier distance) {
        return forward(drive, () -> -distance.getAsDouble());
    }

    public static Command strafeLeft(Drive drive, DoubleSupplier distance) {
        return Drive.followPath(
                drive,
                () -> drive.getPathBuilder()
                        .addPath(new Path(new BezierLine(
                                drive.getDesiredPose().getPose(),
                                new Pose(
                                        drive.getDesiredPose().getPose().getX() - Math.sin(drive.getDesiredPose().getPose().getHeading()) * distance.getAsDouble(),
                                        drive.getDesiredPose().getPose().getY() + Math.cos(drive.getDesiredPose().getPose().getHeading()) * distance.getAsDouble(),
                                        drive.getPose().getHeading()))))
                        .setLinearHeadingInterpolation(drive.getDesiredPose().getPose().getHeading(), drive.getDesiredPose().getPose().getHeading())
                        .build());
    }

    public static Command strafeRight(Drive drive, DoubleSupplier distance) {
        return strafeLeft(drive, () -> -distance.getAsDouble());
    }

    public static Command turn(Drive drive, DoubleSupplier angle) {
        return Drive.followPath(
                drive,
                () -> drive.getPathBuilder()
                        .addPath(new Path(new BezierPoint(drive.getDesiredPose().getPose())))
                        .setConstantHeadingInterpolation(drive.getDesiredPose().getPose().getHeading() + Math.toRadians(angle.getAsDouble()))
                        .build()).until(drive::headingIsFinished).andThen(Commands.waitSeconds(0.5));
    }

    public static Command forward(Drive drive, double distance) {
        return forward(drive, () -> distance);
    }

    public static Command backward(Drive drive, double distance) {
        return backward(drive, () -> distance);
    }

    public static Command strafeLeft(Drive drive, double distance) {
        return strafeLeft(drive, () -> distance);
    }

    public static Command strafeRight(Drive drive, double distance) {
        return strafeRight(drive, () -> distance);
    }

    public static Command turn(Drive drive, double angle) {
        return turn(drive, () -> angle);
    }

    public static Command setPoseStorage(Drive drive, Supplier<Pose> pose) {
        return Commands.runOnce(() -> {
            PoseStorage.currentPose = pose.get();
        }, drive);
    }

    public static double signSquare(double num) {
        return num * num * Math.signum(num);
    }
}
