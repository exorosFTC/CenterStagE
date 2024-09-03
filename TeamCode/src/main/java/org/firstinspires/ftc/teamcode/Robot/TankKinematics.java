package org.firstinspires.ftc.teamcode.Robot;


import org.firstinspires.ftc.teamcode.Pose;

public class TankKinematics {
    private double TRACK_WIDTH;

    public TankKinematics(double trackWidth) {
        this.TRACK_WIDTH = trackWidth;
    }

    public ChassisState inverse(Pose pose) {
        double left_speed = pose.x + this.TRACK_WIDTH * 0.5 * pose.head;
        double right_speed = pose.x - this.TRACK_WIDTH * 0.5 * pose.head;

        return new ChassisState(left_speed, right_speed);
    }

}
