package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group="A")
public class RobotCentric extends MainOp {
    @Override
    public void onUpdate() throws InterruptedException {
        super.onUpdate();
        robot.mecanum.drive(x * speed, y * speed, rot * speed);
    }
}
