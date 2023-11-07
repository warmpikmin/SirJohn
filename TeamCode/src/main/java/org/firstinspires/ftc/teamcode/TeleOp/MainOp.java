package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Bots.SirJohn;

@TeleOp
public class MainOp extends BaseOpMode{
    public SirJohn robot;
    public double x, y, rot, speed, xModifier, yModifier;
    public boolean slowmode;

    @Override
    protected Robot setRobot() {
        this.robot = new SirJohn();
        return this.robot;
    }
    @Override
    protected boolean setTeleOp() {
        return true;
    }
    @Override
    public void onInit() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    @Override
    public void onStart() throws InterruptedException {

    }

    @Override
    public void onUpdate() throws InterruptedException {
        speed = (gamepad1.left_bumper ? 0.25 : (gamepad1.right_bumper || slowmode ? 0.5 : 1)) * (gamepad1.left_stick_button ? 1 : 0.65);
        x = gamepad1.left_stick_x;
        y = - gamepad1.left_stick_y;
        rot = gamepad1.right_stick_x;

        if (gamepad1.dpad_up) {
            y = 1;
        } else if (gamepad1.dpad_down) {
            y = - 1;
        }

        if (gamepad1.dpad_right) {
            x = 1;
        } else if (gamepad1.dpad_left) {
            x = - 1;
        }

        robot.mecanum.drive(x * speed, y * speed, rot * speed);
        telemetry.update();
    }
}
