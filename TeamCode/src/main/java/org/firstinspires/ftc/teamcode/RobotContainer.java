package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.auto.BlueAuto;
import org.firstinspires.ftc.teamcode.lib.wpilib.CommandGamepad;
import org.firstinspires.ftc.teamcode.opmodes.OpModeConstants;
import org.firstinspires.ftc.teamcode.subsystems.conveyor.Conveyor;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;

public class RobotContainer {
    private final Drive drive;
    private final Shooter shooter;
    private final Intake intake;
    private final Conveyor conveyor;

    private final CommandGamepad driverOneController;

    public RobotContainer(HardwareMap hwMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, OpModeConstants autoNum) {
        drive = new Drive(hwMap, telemetry);
        shooter = new Shooter(hwMap, telemetry);
        intake = new Intake(hwMap);
        conveyor = new Conveyor(hwMap);

        driverOneController = new CommandGamepad(gamepad1);

        if (autoNum == OpModeConstants.TELEOP) {
            setDefaultCommands();
            configureButtonBindings();
        } else {
            getAutoCommand(autoNum);
        }
    }

    public void setDefaultCommands(){
        drive.setDefaultCommand(
                Commands.run(() -> drive.drive(
                        () -> -driverOneController.getLeftY(),
                        () -> -driverOneController.getLeftX(),
                        () -> -driverOneController.getRightX()
                ), drive)
        );
    }

    public void configureButtonBindings() {

        // configure your command bindings to your controller like this

        driverOneController.a().onTrue(
                Commands.sequence(
                        Shooter.setVelocity(shooter, () -> ShooterConstants.longShot).withTimeout(0)
                )
        );
        //onTrue = pressing smt onece   whileTrue = holding
        driverOneController.leftTrigger().whileTrue(
                Commands.sequence(
                         Intake.setPower(intake, () -> 1)
                )
        );

        driverOneController.leftBumper().whileTrue(
                Commands.sequence(
                        Intake.setPower(intake, () -> -1)
                )
        );
    }

    public Command getAutoCommand(OpModeConstants auto) {
        return switch (auto) {
            case BLUE_AUTO -> BlueAuto.getBlueAutoCommand(shooter);
            // case RED_AUTO -> RedAuto.getRedAutoCommand(shooter);
            default -> Commands.none();
        };
    }
}