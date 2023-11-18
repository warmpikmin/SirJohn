package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Bots.SirJohn;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Outtake;

@TeleOp
@Disabled
public class MainOp extends BaseOpMode {
    public SirJohn robot;
    public double x, y, rot, speed;
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
            if (robot.intake.arm.getTargetPosition() == Intake.backward) {
                robot.outtake.toMiddle();
            }
        };

        gamepadListener2.b.onRelease = () -> {
            //figure out what to do in the cases of 0 or 1 pixel already on outtake
            //bc that will determine whether pins should open or not before flipping.

            robot.intake.toggleArm();

        };

        gamepadListener2.x.onRelease = () -> {
            robot.outtake.toggleFlip();
        };


    }

    @Override
    public void onUpdate() throws InterruptedException {
        speed = (gamepad1.left_bumper ? 0.25 : (gamepad1.right_bumper || slowmode ? 0.5 : 1)) * (gamepad1.left_stick_button ? 1 : 0.75);
        x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        rot = gamepad1.right_stick_x;

        if (gamepad1.dpad_up) {
            y = 1;
        } else if (gamepad1.dpad_down) {
            y = -1;
        }

        if (gamepad1.dpad_right) {
            x = 1;
        } else if (gamepad1.dpad_left) {
            x = -1;
        }

        if (gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) {
            if (robot.slides.getCurrentPosition() < robot.slides.LOWER_BOUND) {
                robot.slides.move(robot.slides.LOWER_BOUND + 10);
            } else if (robot.slides.getCurrentPosition() > robot.slides.UPPER_BOUND) {
                robot.slides.move(robot.slides.UPPER_BOUND - 10);
            } else {
                robot.slides.move((int) ((gamepad2.right_trigger - gamepad2.left_trigger) * 100) + robot.slides.getCurrentPosition(), gamepad2.right_trigger - gamepad2.left_trigger);
            }
        }

        telemetry.update();
    }
}
