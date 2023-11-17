package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Bots.SirJohn;
import org.firstinspires.ftc.teamcode.Components.Outtake;

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
        robot.camera.isInit = false;

        gamepadListener1.start.onRelease = () -> {
            slowmode = !slowmode;
        };

        gamepadListener2.start.onRelease = () -> {
            robot.crossbow.toggleLauncher();
        };

        gamepadListener2.a.onRelease = () -> {
            robot.intake.toggleClaw();
        };

        gamepadListener2.b.onRelease = () -> {
            //figure out what to do in the cases of 0 or 1 pixel already on outtake
            //bc that will determine whether pins should open or not before flipping.

            robot.intake.openClaw();
            robot.outtake.unFlip();
        };

        gamepadListener2.x.onRelease = () -> {
            robot.outtake.toggleFlip();
        };

        gamepadListener2.dd.onRelease = () -> {
            robot.slides.toZero();
        };
        gamepadListener2.dl.onRelease = () -> {
            robot.slides.toPlace();
        };
        gamepadListener2.du.onRelease = () -> {
            robot.slides.toUpperPlace();
        };

    }

    @Override
    public void onUpdate() throws InterruptedException {
        speed = (gamepad1.left_bumper ? 0.25 : (gamepad1.right_bumper || slowmode ? 0.5 : 1)) * (gamepad1.left_stick_button ? 1 : 0.75);
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

        //.014 is the value from last year's arm movement-- does this need to change?
        if (gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) {
            if (robot.slides.getCurrentPosition() < robot.slides.LOWER_BOUND) {
               robot.slides.move(robot.slides.LOWER_BOUND + (int) (robot.slides.PULSES_PER_REVOLUTION * 0.014));
            } else if (robot.slides.getCurrentPosition() > robot.slides.UPPER_BOUND) {
                robot.slides.move(robot.slides.UPPER_BOUND - (int) (robot.slides.PULSES_PER_REVOLUTION * 0.014));
            } else {
                robot.slides.move((int) ((gamepad2.right_trigger - gamepad2.left_trigger) * robot.slides.PULSES_PER_REVOLUTION * 0.5) + robot.slides.getCurrentPosition());
            }
        }

        robot.mecanum.drive(x * speed, y * speed, rot * speed);
        telemetry.update();
    }
}
