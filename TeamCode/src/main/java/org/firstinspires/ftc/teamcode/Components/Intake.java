package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.Component;

public class Intake implements Component {
    private Servo claw;
    public DcMotor arm;
    public double armPower;
    public Telemetry telemetry;
    public Intake(String deviceName, HardwareMap hardwareMap, String arm, Telemetry telemetry){
        claw = hardwareMap.get(Servo.class, deviceName);
        this.telemetry = telemetry;
        this.arm = hardwareMap.dcMotor.get(arm);
        this.arm.setDirection(DcMotorSimple.Direction.FORWARD);
        this.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        arm.setPower(armPower);

    }

    @Override
    public String getTelemetry() {
        return null;
    }
}
