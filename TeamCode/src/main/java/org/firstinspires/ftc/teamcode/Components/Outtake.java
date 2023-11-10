package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.Component;

public class Outtake implements Component {
    private Servo spin;
    private Servo pins;
    public boolean spinIsSpun;
    public Pins pinsPlacement;
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
    public enum Pins{
        CLOSED,
        MID,
        OPEN;
    }

    public void closedPins(){
        pinsPlacement = Pins.CLOSED;
        //updatePosPins();
    }
    public void midPins(){
        pinsPlacement = Pins.MID;
        //updatePosPins();
    }
    public void openPins(){
        pinsPlacement = Pins.OPEN;
        //updatePosPins();
    }
    public void togglePins(){
        //switch between closed, mid, and open
    }
    public void updatePosPins(){
        //change position
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
        telemetry.addData("pinIsPinned", pins);
        telemetry.addData("spinsPos",spin.getPosition());
        telemetry.addData("spinIsSpun",spinIsSpun);
        return null;
    }
}
