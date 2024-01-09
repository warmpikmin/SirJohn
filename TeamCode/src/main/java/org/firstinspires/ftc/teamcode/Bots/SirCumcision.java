package org.firstinspires.ftc.teamcode.Bots;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Components.LEDs;



public class SirCumcision extends Robot {

    public LEDs leds;

    @Override
    protected void mapHardware(HardwareMap hardwareMap, Telemetry telemetry, BaseOpMode opMode, boolean isTeleOp) {
        this.leds = new LEDs("Pb",hardwareMap,telemetry);
        addComponents(leds);

    }
}
