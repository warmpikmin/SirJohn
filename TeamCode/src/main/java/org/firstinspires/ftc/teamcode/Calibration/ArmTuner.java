package org.firstinspires.ftc.teamcode.Calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Utils.GamepadListener;

@TeleOp

public class ArmTuner extends LinearOpMode {
    public Intake intake;
    public GamepadListener gamepadListener = new GamepadListener();
    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake("arm","claw", hardwareMap,telemetry, true,0,5,90,0,1);
        intake.init();
        waitForStart();
        intake.start();
        gamepadListener.a.onRelease = () -> {
            intake.toggleArm();
        };
        gamepadListener.b.onRelease = () -> {
            intake.toggleClaw();
        };
        while (opModeIsActive()) {
            gamepadListener.update(gamepad1);
            intake.update();
        }
    }
}
