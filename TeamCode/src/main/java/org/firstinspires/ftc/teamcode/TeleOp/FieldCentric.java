package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(group="A")
public class FieldCentric extends MainOp {
    @Override
    public void onUpdate() throws InterruptedException {
        super.onUpdate();
        if (gamepad1.back) {
            robot.imu.resetYaw();
        }

        robot.mecanum.fieldCentricDrive(
            x * speed,
            y * speed,
            rot * speed,
            robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
        );
    }
}
