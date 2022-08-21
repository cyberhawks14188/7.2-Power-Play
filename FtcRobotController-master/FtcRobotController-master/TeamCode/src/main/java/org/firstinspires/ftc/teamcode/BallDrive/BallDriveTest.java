package org.firstinspires.ftc.teamcode.BallDrive;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp
@Config

public class BallDriveTest extends LinearOpMode{


    BallDriveHardware robot = new BallDriveHardware();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    @Override

    public void runOpMode(){
        robot.init(hardwareMap);


        waitForStart();

        while (opModeIsActive()){

            robot.MotorLL.setPower(-gamepad1.left_stick_y);
            robot.MotorLR.setPower(gamepad1.left_stick_y);

            telemetry.addData("LL Current", robot.MotorLL.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("LR Current", robot.MotorLR.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("RL Current", robot.MotorRL.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("RR Current", robot.MotorRR.getCurrent(CurrentUnit.MILLIAMPS));
            dashboardTelemetry.addData("LL Current", robot.MotorLL.getCurrent(CurrentUnit.MILLIAMPS));
            dashboardTelemetry.addData("LR Current", robot.MotorLR.getCurrent(CurrentUnit.MILLIAMPS));
            dashboardTelemetry.addData("RL Current", robot.MotorRL.getCurrent(CurrentUnit.MILLIAMPS));
            dashboardTelemetry.addData("RR Current", robot.MotorRR.getCurrent(CurrentUnit.MILLIAMPS));
            dashboardTelemetry.update();

            telemetry.update();

        }

    }


}