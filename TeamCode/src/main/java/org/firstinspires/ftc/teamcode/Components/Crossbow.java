package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.Component;

public class Crossbow implements Component {

    private Servo launch;
    private boolean launched = false;
    Telemetry telemetry;
    public Crossbow(String deviceName, HardwareMap hardwareMap, Telemetry telemetry){
        launch = hardwareMap.get(Servo.class, deviceName);
        this.telemetry = telemetry;
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
    //fluently instigating rapid ejection
    //TODO figure out if this value works
    public void FIRE(){
        launch.setPosition(0.5);
        launched = true;
    }
    public void reset(){
        launch.setPosition(0);
        launched = false;
    }

    @Override
    public String getTelemetry() {
        telemetry.addData("Position", launch.getPosition());
        telemetry.addData("Launched", launched);
        return null;
    }
}
