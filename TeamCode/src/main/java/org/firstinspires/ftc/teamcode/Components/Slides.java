package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.Component;

public class Slides implements Component {
    public final DcMotor leftArm;
    public final DcMotor rightArm;



    public double PULSES_PER_REVOLUTION;
    public int LOWER_BOUND;
    public int UPPER_BOUND;
    public int ZERO_POSITION;
    public int PLACE_POSITION;
    public int UPPER_PLACE_POSITION;
    public static int targetPosition = 0;

    public boolean isTeleOp;

    public double error, power = 0;

    public double kG;

    public double MotorPower;
    public int TotalTicks, StartingPosition;
    Telemetry telemetry;


    public Slides(String leftName, String rightName, HardwareMap hardwareMap, Telemetry telemetry, boolean isTeleOp,double lowerBound, double upperBound, double zeroPosition, double placePosition, double upperPlacePosition, double kG) {
        leftArm = hardwareMap.get(DcMotor.class, leftName);
        rightArm = hardwareMap.get(DcMotor.class,rightName);
        this.PULSES_PER_REVOLUTION = 145.1;
        this.LOWER_BOUND = (int) (lowerBound * PULSES_PER_REVOLUTION);
        this.UPPER_BOUND = (int) (upperBound * PULSES_PER_REVOLUTION);
        this.ZERO_POSITION = (int) (zeroPosition * PULSES_PER_REVOLUTION);
        this.PLACE_POSITION = (int) (placePosition * PULSES_PER_REVOLUTION);
        this.UPPER_PLACE_POSITION = (int) (upperPlacePosition * UPPER_PLACE_POSITION);
        this.kG = kG;
        this.isTeleOp = isTeleOp;
        rightArm.setDirection(DcMotor.Direction.REVERSE);
        leftArm.setDirection(DcMotor.Direction.FORWARD);
        this.telemetry = telemetry;
    }



    @Override
    public void init() {
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        move(ZERO_POSITION);


    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        telemetry.addData("Position", getCurrentPosition());
        if (isTeleOp) {
            if (isBusy()) {
                setPower(MotorPower);
//                setPower(((-4.0 * MotorPower) / Math.pow(TotalTicks, 2.0)) * Math.pow(TotalTicks / 2.0 - getCurrentPosition(), 2.0) + MotorPower);
            } else {
                setPower(0);
                move(getTargetPosition());
            }
        } else {
            if (getCurrentPosition() != getTargetPosition()) move(getTargetPosition());
        }
    }

    @Override
    public String getTelemetry() {
        return null;
    }

    public void move(int position) {
        move(position, 1);
    }

    public void move(int position, double motorPower) {
        leftArm.setTargetPosition(position);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setTargetPosition(position);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorPower = motorPower;
        TotalTicks = position;
        StartingPosition = getCurrentPosition();
//        if (!isTeleOp) {
//            while (isBusy()) {
//                setPower(MotorPower);
//            }
//            setPower(0);
//        }
    }

    public void setPower(double motorPower) {
        leftArm.setPower(motorPower);
        rightArm.setPower(motorPower);
    }

    public boolean isBusy() {
        return leftArm.isBusy();
    }

    public int getCurrentPosition() {
        return leftArm.getCurrentPosition();
    }

    public int getTargetPosition() {
        return leftArm.getTargetPosition();
    }
}