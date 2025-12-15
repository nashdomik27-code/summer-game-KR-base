package org.firstinspires.ftc.teamcode.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;

public class BlueAuto {

    public static Command getBlueAutoCommand(Shooter shooter) {
        return Commands.sequence(
                Shooter.setVelocity(shooter, () -> ShooterConstants.midShot).withTimeout(0),

                Commands.waitSeconds(1),

                Shooter.setVelocity(shooter, () -> ShooterConstants.longShot).withTimeout(0)
        );
    }

}
