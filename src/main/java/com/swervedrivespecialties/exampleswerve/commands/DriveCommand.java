package com.swervedrivespecialties.exampleswerve.commands;

import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.frcteam2910.common.robot.Utilities;

public class DriveCommand extends Command {

    public DriveCommand() {
        requires(DrivetrainSubsystem.getInstance());
    }

    @Override
    protected void execute() {
        final double joystickDampen = 1.0;
        final double driveDeadband = 0.09;
        final double turnDeadband = 0.15;

        boolean fieldOrient = !Robot.getOi().getJoy1().getRawButton(3);

        double forward = Robot.getOi().getJoy1().getRawAxis(1);
        forward = Utilities.deadband(forward, driveDeadband);
        // Square the forward stick
        forward = Math.copySign(Math.pow(forward, 2.0), forward) * joystickDampen;

        double strafe = Robot.getOi().getJoy1().getRawAxis(0);
        strafe = Utilities.deadband(strafe, driveDeadband);
        // Square the strafe stick
        strafe = Math.copySign(Math.pow(strafe, 2.0), strafe) * joystickDampen;

        double rotation = Robot.getOi().getJoy0().getRawAxis(2);
        SmartDashboard.putNumber("joy0 axis 3 raw", rotation);

        if (-0.25 < rotation && rotation < -0.008) {
          rotation = 0.0;
        } else if (rotation < -0.25) {
          rotation = Robot.linearMap(rotation, -1, -0.25, -1, -0.1);
        } else if (rotation > -0.008) {
          rotation = Robot.linearMap(rotation, 0, 1, 0.1, 1);
        } 
        SmartDashboard.putNumber("joy0 axis 3 mapped", rotation);

        // // -0.25 to -0.008
        // rotation += 0.15;
        // rotation = Utilities.deadband(rotation, turnDeadband);
        // Square the rotation stick
        rotation = Math.copySign(Math.pow(rotation, 2.0), rotation) * joystickDampen;
        SmartDashboard.putNumber("joy0 axis 3 adjusted", rotation);

        DrivetrainSubsystem.getInstance().drive(new Translation2d(forward, strafe), rotation, fieldOrient);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
