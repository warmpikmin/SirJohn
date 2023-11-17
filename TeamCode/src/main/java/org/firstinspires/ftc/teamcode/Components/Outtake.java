package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.Component;

public class Outtake implements Component {
    public Servo spin;
    //private Servo pins;
    public boolean spinPos;
    //public Pins pinsPos;
    //public double pinsIn;
    //public double pinsMid;
    //public double pinsOut;
    public double spun;
    public double middle;
    public double unSpun;
    Telemetry telemetry;
    public Outtake(String spinName, HardwareMap hardwareMap, Telemetry telemetry, double spun, double unSpun,double middle){
        //pins = hardwareMap.get(Servo.class, pinsName);
        spin = hardwareMap.get(Servo.class, spinName);
        this.telemetry = telemetry;
        //this.pinsIn = pinsIn;
        //this.pinsMid = pinsMid;
        //this.pinsOut = pinsOut;
        this.spun = spun;
        this.unSpun= unSpun;
        this.middle = middle;
    }
    @Override
    public void init() {
        unFlip();
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

    }
    /*

    public enum Pins{
        CLOSED,
        MID,
        OPEN;
    }

    public void closePins(){
        pinsPos = Pins.CLOSED;
        updatePosPins();
    }
    public void midPins(){
        pinsPos = Pins.MID;
        updatePosPins();
    }
    public void openPins(){
        pinsPos = Pins.OPEN;
        updatePosPins();
    }
    public void togglePins(){
        pinsPos = (pinsPos != Pins.OPEN ? Pins.OPEN : Pins.CLOSED);
        updatePosPins();
    }
    public void updatePosPins(){
        pins.setPosition(pinsPos == Pins.CLOSED ? pinsIn : pinsPos == Pins.MID ? pinsMid : pinsOut);
    }

    */
    public void flip(){
        spinPos = true;
        updatePosFlip();
    }
    public void unFlip(){
        spinPos = false;
        updatePosFlip();
    }
    public void toggleFlip(){
        spinPos = !spinPos;
        updatePosFlip();
    }
    public void toMiddle(){
        spin.setPosition(middle);
    }
    public void updatePosFlip(){
        spin.setPosition(spinPos ? spun : unSpun);
    }

    @Override
    public String getTelemetry() {
        //telemetry.addData("pinsPos",pins.getPosition());
        //telemetry.addData("pinIsPinned", pins);
        telemetry.addData("spinsPos",spin.getPosition());
        telemetry.addData("spinIsSpun",spinPos);
        return null;
    }
}
