package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Pose;

public class Chassis {
    private LinearOpMode opMode;

    private DcMotorEx left_motor;
    private DcMotorEx right_motor;

    private TankKinematics kinematics;

    public Chassis(LinearOpMode opMode, double trackWidth)
    {
        this.opMode = opMode;

        left_motor = opMode.hardwareMap.get(DcMotorEx.class, HardwareNames.leftWheel);
        right_motor = opMode.hardwareMap.get(DcMotorEx.class, HardwareNames.rightWheel);

        this.kinematics = new TankKinematics(trackWidth);
    }

    public void drive(Pose velocity) {
        ChassisState state = kinematics.inverse(velocity);
        setWheelPowers(state.leftSpeed, state.rightSpeed);
    }

    public void setWheelPowers(double left, double right) {
        left_motor.setPower(left);
        right_motor.setPower(right);
    }

}
