package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Pose;
import org.firstinspires.ftc.teamcode.Robot.Core;


@Config
@TeleOp(name = "TELE_OP", group = "main")
public class MainTeleOp extends LinearOpMode {
    private Core robot;
    private boolean toggle = false;
    private boolean toggleArm = false;
    private double sensitivity = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Core(this);
        robot.setArmState(Core.WristState.INTERMEDIARY);
        robot.drone.load();

        waitForStart();

        while (opModeIsActive()) {
            robot.update();

            Pose velocity = new Pose(
                    expon(robot.g1.getRightY()) * sensitivity,
                    0,
                    expon(robot.g1.getLeftY()) * sensitivity
            );

            if (robot.g1.wasJustPressed(GamepadKeys.Button.X))
                robot.drone.fire();

            if (robot.g1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER))
                if (robot.getArmState() == Core.WristState.PICK)
                    robot.setArmState(Core.WristState.INTERMEDIARY);
                else {
                    robot.setArmState(Core.WristState.PICK);
                    toggle = false; }

            if (robot.g1.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER))
                if (robot.getArmState() != Core.WristState.INTERMEDIARY)
                robot.setArmState(Core.WristState.INTERMEDIARY);

            if (robot.g1.wasJustPressed(GamepadKeys.Button.B)) {
                toggle = !toggle;

                if (toggle) robot.setClawState(Core.ClawState.OPEN);
                else robot.setClawState(Core.ClawState.CLOSE);
            }

            if (robot.g1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                toggleArm = !toggleArm;

                if (toggleArm) robot.setArmState(Core.WristState.SCORE);
                else robot.setArmState(Core.WristState.PICK);
            }

            robot.drive((velocity));

            telemetry.addData("State: ", robot.getArmState());
            telemetry.update();
        }
    }

    private double expon(double x) {
        return x * x * x;
    }

}
