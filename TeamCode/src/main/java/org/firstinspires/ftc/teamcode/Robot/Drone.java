package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Drone {
    private double launch = 0.3;
    private double load = 0.65;

    private Servo servo;

    public Drone(LinearOpMode opMode) {
        servo = opMode.hardwareMap.get(Servo.class, "drone_launcher");
    }

    public void fire() { servo.setPosition(launch); }

    public void load() { servo.setPosition(load); }
}
