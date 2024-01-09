package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Bots.SirCumcision;
import org.firstinspires.ftc.teamcode.Components.LEDs;

@TeleOp
public class Mohel extends BaseOpMode {
    public SirCumcision robot;

    @Override
    public void onUpdate() throws InterruptedException {
        robot.leds.getTelemetry();
        gamepadListener1.x.onRelease = () -> {
            robot.leds.setPatternBlue();
        };
        gamepadListener1.back.onRelease = () -> {
            robot.leds.iLoveSimonApley();
        };
        gamepadListener1.y.onRelease = () -> {
            robot.leds.setPatternGay();
        };
        gamepadListener1.start.onRelease = () -> {
            robot.leds.iLoveGabiNatenshon();
    };
        }

    @Override
    protected Robot setRobot() {
        this.robot = new SirCumcision();
        return this.robot;
    }

    @Override
    protected boolean setTeleOp() {
        return false;
    }
}
