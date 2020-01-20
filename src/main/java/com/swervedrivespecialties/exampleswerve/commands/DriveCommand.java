package com.swervedrivespecialties.exampleswerve.commands;

import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import org.frcteam2910.common.robot.Utilities;

public class DriveCommand extends Command {

    public DriveCommand() {
        requires(DrivetrainSubsystem.getInstance());
    }

    @Override
    protected void execute() {
        final double joystickDampen = 1.0;
        final double deadband = 0.09;

        double forward = -Robot.getOi().getJoy2().getRawAxis(1);
        forward = Utilities.deadband(forward, deadband);
        // Square the forward stick
        forward = Math.copySign(Math.pow(forward, 2.0), forward) * joystickDampen;

        double strafe = -Robot.getOi().getJoy2().getRawAxis(0);
        strafe = Utilities.deadband(strafe, deadband);
        // Square the strafe stick
        strafe = Math.copySign(Math.pow(strafe, 2.0), strafe) * joystickDampen;

        double rotation = Robot.getOi().getJoy1().getRawAxis(2);
        rotation = Utilities.deadband(rotation, deadband);
        // Square the rotation stick
        rotation = Math.copySign(Math.pow(rotation, 2.0), rotation) * joystickDampen;

        DrivetrainSubsystem.getInstance().drive(new Translation2d(forward, strafe), rotation, true);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
