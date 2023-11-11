package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.Component;

public class Outtake implements Component {
    private Servo spin;
    private Servo pins;
    public boolean spinPos;
    public Pins pinsPos;
    public double pinsClosed;
    public double pinsMid;
    public double pinsOpen;
    public double spun;
    public double unSpun;
    Telemetry telemetry;
    public Outtake(String pinsName, String spinName, HardwareMap hardwareMap, Telemetry telemetry, double pinsClosed, double pinsMid, double pinsOpen, double spun, double unSpun){
        pins = hardwareMap.get(Servo.class, pinsName);
        spin = hardwareMap.get(Servo.class, spinName);
        this.telemetry = telemetry;
        this.pinsClosed = pinsClosed;
        this.pinsMid = pinsMid;
        this.pinsOpen = pinsOpen;
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
        pins.setPosition(pinsPos == Pins.CLOSED ? pinsClosed : pinsPos == Pins.MID ? pinsMid : pinsOpen);
    }
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
    public void updatePosFlip(){
        spin.setPosition(spinPos ? spun : unSpun);
    }

    @Override
    public String getTelemetry() {
        telemetry.addData("pinsPos",pins.getPosition());
        telemetry.addData("pinIsPinned", pins);
        telemetry.addData("spinsPos",spin.getPosition());
        telemetry.addData("spinIsSpun",spinPos);
        return null;
    }
}
