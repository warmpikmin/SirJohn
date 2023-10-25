package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.Component;
@Config
public class Hanger implements Component{
    private final DcMotor hanger;
    public double PULSES_PER_REVOLUTION;
    public int INIT;
    public int UP;
    public int DOWN;
    public static int targetPosition = 0;
    public boolean isTeleOp;

    public double error;
    Telemetry telemetry;

    public Hanger(
            String hangerName,
            HardwareMap hardwareMap,
            Telemetry telemetry,
            boolean isTeleOp,
            double init,
            double up,
            double down
    ){
        hanger = hardwareMap.get(DcMotor.class, hangerName);

        hanger.setDirection(DcMotorSimple.Direction.FORWARD);

        this.PULSES_PER_REVOLUTION = 2786.2;
        this.INIT = (int) (init * PULSES_PER_REVOLUTION);
        this.UP = (int) (up * PULSES_PER_REVOLUTION);
        this.DOWN = (int) (down * PULSES_PER_REVOLUTION);
        this.isTeleOp = isTeleOp;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void init() {
        hanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hanger.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        error = targetPosition - getCurrentPosition();

        //unfinished obviously

    }

    @Override
    public String getTelemetry() {
        telemetry.addData("HangerPosition", getCurrentPosition());
        telemetry.addData("HangerTarget", targetPosition);
        telemetry.addData("HangerError", error);
        return null;
    }

    public void toInit() {
        move(INIT);
    }

    public void toForward() {
        move(UP);
    }

    public void toBackward() {
        move(DOWN);
    }

    public void toggle() {
        if (getCurrentPosition() + error == DOWN) {
            toForward();
        } else if (getCurrentPosition() + error == UP) {
            toBackward();
        }
    }

    public void move(int position) {
        //unfinished
    }

    public void setPower(double motorPower) {
        if (motorPower > 1) motorPower = 1;
        hanger.setPower(motorPower);
    }

    public int getCurrentPosition() {
        return hanger.getCurrentPosition();
    }
}
