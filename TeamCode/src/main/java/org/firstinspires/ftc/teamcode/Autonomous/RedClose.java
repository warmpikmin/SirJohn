package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Bots.SirJohn;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.RRMecanum;

@Autonomous
public class RedClose extends BaseOpMode {
    public SirJohn robot;
    public RRMecanum drive;

    public Trajectory left, right, center;

    @Override
    protected Robot setRobot() {
        return null;
    }

    @Override
    protected boolean setTeleOp() {
        return false;
    }
}
