package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.Component;

public class Outtake implements Component {
    private Servo spin;
    private Servo pins;
    public boolean spinIsSpun;
    public boolean pinIsPinned;
    public double pinRelease;
    public double pinHold;
    public double spun;
    public double unSpun;
    Telemetry telemetry;
    public Outtake(String pinsName, String spinName, HardwareMap hardwareMap, Telemetry telemetry, double pinRelease, double pinHold, double spun, double unSpun){
        pins = hardwareMap.get(Servo.class, pinsName);
        spin = hardwareMap.get(Servo.class, spinName);
        this.telemetry = telemetry;
        this.pinRelease = pinRelease;
        this.pinHold = pinHold;
        this.spun = spun;
        this.unSpun= unSpun;
    }
    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

    }

    public void closePins(){
        //close
    }
    public void openPins(){
        //open
    }
    public void togglePins(){
        //change
    }
    public void flip(){
        spinIsSpun = true;
        updatePosFlip();
    }
    public void unFlip(){
        spinIsSpun = false;
        updatePosFlip();
    }
    public void toggleFlip(){
        spinIsSpun = !spinIsSpun;
        updatePosFlip();
    }
    public void updatePosFlip(){
        if(spinIsSpun){
            spin.setPosition(spun);
        }else{
            spin.setPosition(unSpun);
        }
    }

    @Override
    public String getTelemetry() {
        telemetry.addData("pinsPos",pins.getPosition());
        telemetry.addData("pinIsPinned", pinIsPinned);
        telemetry.addData("spinsPos",spin.getPosition());
        telemetry.addData("spinIsSpun",spinIsSpun);
        return null;
    }
}
