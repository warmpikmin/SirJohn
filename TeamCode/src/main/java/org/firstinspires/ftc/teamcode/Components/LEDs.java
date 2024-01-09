package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.Component;

public class LEDs implements Component {
    private RevBlinkinLedDriver LED;
    private Telemetry telemetry;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    public LEDs(String LEDName, HardwareMap hardwareMap,Telemetry telemetry){
        this.telemetry = telemetry;
        this.LED = hardwareMap.get(RevBlinkinLedDriver.class,LEDName );
        pattern = RevBlinkinLedDriver.BlinkinPattern.GOLD;

    }
    public void setPatternBlue(){
        this.changePattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }
    public void setPatternGay(){
        this.changePattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
    }
    public void iLoveSimonApley(){
        this.changePattern(RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE);
    }
    public void iLoveGabiNatenshon(){
        this.changePattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
    }
    @Override
    public void init() {

    }
    public void changePattern(RevBlinkinLedDriver.BlinkinPattern newPattern){
        pattern = newPattern;
        LED.setPattern(pattern);
    }
    public String getPattern(){
        return pattern.toString();
    }
    @Override
    public void start() {

    }

    @Override
    public void update() {

    }

    @Override
    public String getTelemetry() {
        return "Pattern: "+this.getPattern();
    }
}
