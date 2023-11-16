package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.Component;
@Config
public class Intake implements Component {
    private Servo claw;
    public DcMotor arm;

    Telemetry telemetry;
    public double PULSES_PER_REVOLUTION;

    public int FORWARD;

    public double closed;
    public double open;
    public static int forward;
    public static int backward;
    public static int targetPosition = 0;
    public boolean isClosed;
    public boolean isTeleOp, forcePosition;
    public double error, prevError = 0, time, prevTime = System.nanoTime() * 1e-9d, power;
    public static double kP = 0.005, kD = 0.00001, kG = 0.1;
    public Intake(
            String armName,
            String clawName,
            HardwareMap hardwareMap,
            Telemetry telemetry,
            boolean isTeleOp,
            double init,
            int forward,
            int backward,
            double closed,
            double open
    ) {
        this.claw = hardwareMap.get(Servo.class, clawName);
        this.arm = hardwareMap.get(DcMotor.class, armName);

        this.telemetry = telemetry;
        this.closed = closed;
        this.open = open;
        Intake.forward = forward;
        Intake.backward = backward;
        this.PULSES_PER_REVOLUTION = 384.5;
        this.isTeleOp = isTeleOp;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    @Override
    public void init() {
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        update(forcePosition);
        getTelemetry();
        telemetry.update();

    }
    public void update(boolean forcePosition){
        error = targetPosition - getCurrentPosition();
        time = System.nanoTime() * 1e-9d;
        this.forcePosition = forcePosition;
//        if (forcePosition || arm.getCurrentPosition() > (2.731 * arm.PULSES_PER_REVOLUTION)) {
        power = (kP * error) + (kD * -(error - prevError) / (time - prevTime)) + (kG * Math.cos(Math.toRadians(targetPosition * (PULSES_PER_REVOLUTION / 360))));
//        }
        if (forcePosition) {
            power = 0;
        }
        if (!isBusy()) {
            this.forcePosition = false;
        }
        setPower(power);
        //setPower(0);
        prevError = error;
        prevTime = time;
    }

    public void closeClaw(){
        isClosed = true;
        updatePos();
    }

    public void openClaw(){
        isClosed = false;
        updatePos();
    }

    public void toggleClaw(){
        isClosed = !isClosed;
        updatePos();
    }

    public void updatePos(){
        claw.setPosition(isClosed ? closed : open);
    }

    @Override
    public String getTelemetry() {
        telemetry.addData("CurrentPosition", getCurrentPosition());
        telemetry.addData("TargetPosition", targetPosition);
        telemetry.addData("arm.getTargetPosition", arm.getTargetPosition());
        telemetry.addData("error", error);
        telemetry.addData("power", arm.getPower());
        telemetry.addData("clawIsClosed", isClosed);
        telemetry.addData("Arm position", arm.getCurrentPosition());
        telemetry.addData("Arm zero power behaviour",arm.getZeroPowerBehavior());
        return null;
    }

    public void toggleArm(){
        if(targetPosition == forward){
            arm.setTargetPosition(backward);
            targetPosition = backward;

        }
        else{
            arm.setTargetPosition(forward);
            targetPosition = forward;

        }
    }



    public void toBackwardForce() {
        update(true);
    }


    //TODO figure out what 2.731 means

    public boolean isBusy() {
        return Math.abs(error) > 10;
    }
    public void setPower(double motorPower) {
        if (motorPower > 1) motorPower = 1;
        arm.setPower(motorPower);
    }
    public int getCurrentPosition() {
        return arm.getCurrentPosition();
    }
}
