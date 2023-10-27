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
public class Slides implements Component {
    private final DcMotor slides;
    public double PULSES_PER_REVOLUTION;
    public int LOWER_BOUND;
    public int ZERO_POSITION;
    public int UPPER_BOUND;
    public int PLACE_POSITION;
    public static int targetPosition = 0;

    public boolean isTeleOp;

    public double error, power;
    Telemetry telemetry;

    public Slides(
            String armName,
            HardwareMap hardwareMap,
            Telemetry telemetry,
            boolean isTeleOp,
            double lowerBound,
            double zeroPosition,
            double upperBound,
            double placePosition
    ) {
        slides = hardwareMap.get(DcMotor.class, armName);

        slides.setDirection(DcMotorSimple.Direction.FORWARD);

        this.PULSES_PER_REVOLUTION = 384.5;
        this.LOWER_BOUND = (int) (lowerBound * PULSES_PER_REVOLUTION);
        this.ZERO_POSITION = (int) (zeroPosition * PULSES_PER_REVOLUTION);
        this.UPPER_BOUND = (int) (upperBound * PULSES_PER_REVOLUTION);
        this.PLACE_POSITION = (int) (placePosition * PULSES_PER_REVOLUTION);

        this.isTeleOp = isTeleOp;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void init() {
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        move(isTeleOp ? ZERO_POSITION : LOWER_BOUND);
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        error = targetPosition - getCurrentPosition();
        setPower(1);
        //set power to something
    }

    @Override
    public String getTelemetry() {
        telemetry.addData("SlidePosition", getCurrentPosition());
        telemetry.addData("SlideTarget", targetPosition);
        telemetry.addData("SlideError", error);
        telemetry.addData("SlidePower", power);
        telemetry.addData("Slides", slides.getCurrentPosition());
        return null;
    }

    public void toZero() {
        move(ZERO_POSITION);
    }

    public void toPlace() {
        move(PLACE_POSITION);
    }

    public void move(int position) {
        targetPosition = position;
    }

    public void setPower(double motorPower) {
        if (motorPower > 1) motorPower = 1;
        slides.setPower(motorPower);
    }

    public boolean isBusy() {
        return Math.abs(error) > 10;
    }

    public int getCurrentPosition() {
        return slides.getCurrentPosition();
    }
}