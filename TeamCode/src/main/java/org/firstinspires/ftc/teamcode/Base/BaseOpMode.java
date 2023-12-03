package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Utils.GamepadListener;

public abstract class BaseOpMode extends LinearOpMode {
    private Robot robot;
    private boolean isTeleOp;

    protected GamepadListener gamepadListener1 = new GamepadListener();
    protected GamepadListener gamepadListener2 = new GamepadListener();

    public enum GameState {
        NOT_INIT,
        IN_INIT,
        IN_START,
        IN_UPDATE,
        STOPPED,
    }

    private GameState gameState;

    @Override
    public void runOpMode() throws InterruptedException {
        gameState = GameState.NOT_INIT;
        robot = setRobot();
        isTeleOp = setTeleOp();
        String possibleTelemetry;

        try {
            robot.mapHardware(hardwareMap, telemetry, this, isTeleOp);

        } catch (NullPointerException e){
            throw new NullPointerException(e + " is null");
        }
        gameState = GameState.IN_INIT;
        robot.components.forEach(Component::init);
        onInit();

        waitForStart();
        gameState = GameState.IN_START;
        onStart();
        robot.components.forEach(Component::start);

        gameState = GameState.IN_UPDATE;
        while (opModeIsActive()) {
            gamepadListener1.update(gamepad1);
            gamepadListener2.update(gamepad2);
            onUpdate();
            robot.components.forEach(Component::update);
            for (Component component : robot.components) {
                possibleTelemetry = component.getTelemetry();
                if (possibleTelemetry != null) {
                    telemetry.addLine(possibleTelemetry);
                }
            }
        }
        gameState = GameState.STOPPED;
    }

    protected double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    public GameState getGameState() {
        return gameState;
    }

    protected abstract Robot setRobot();
    protected abstract boolean setTeleOp();

    public void onInit() throws InterruptedException {}
    public void onStart() throws InterruptedException {}
    public void onUpdate() throws InterruptedException {}

}
