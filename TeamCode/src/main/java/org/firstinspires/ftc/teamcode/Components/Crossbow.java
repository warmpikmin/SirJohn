package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.Component;

public class Crossbow implements Component {

    private Servo launch;
    private boolean launched = false;
    public double firePos;
    public double init;
    Telemetry telemetry;
    public Crossbow(String deviceName, HardwareMap hardwareMap, Telemetry telemetry, double firePos, double init){
        launch = hardwareMap.get(Servo.class, deviceName);
        this.telemetry = telemetry;
        this.init=init;
        this.firePos = firePos;
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
        launched = true;
        updatePosLauncher()
    }
    public void reset(){
        launched = false;
        updatePosLauncher()
    }

    public void toggleLauncher(){
        launched = !launched;
        updatePosLauncher()
    }

    public void updatePosLauncher(){
        launch.setPosition(launched ? firePos : init);
    }

    @Override
    public String getTelemetry() {
        telemetry.addData("Position", launch.getPosition());
        telemetry.addData("Launched", launched);
        return null;
    }
}
