package org.firstinspires.ftc.teamcode.BallDrive;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp
@Config

public class HolonimicTest extends LinearOpMode{


    BallDriveHardware robot = new BallDriveHardware();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    ModernRoboticsI2cGyro gyro    = null;                    // Additional Gyro device

    private double          robotHeading = 0;

    double x,y,z;
    double desiredHeading = 0;
    double finalAngle = 0;
    double finalX, finalY;

    @Override

    public void runOpMode(){
        robot.init(hardwareMap);

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();


        gyro.resetZAxisIntegrator();
        getHeading();

//test
        waitForStart();

        while (opModeIsActive()){

            x = gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;
            z = gamepad1.right_stick_x;

            desiredHeading = Math.atan2(z,y);
            desiredHeading = Math.toDegrees(desiredHeading);

            if(x < .1){
                if(y>0){
                    desiredHeading = 0;
                }else{
                    desiredHeading = 180;
                }
            }

            finalAngle = desiredHeading + getHeading();
            finalX = Math.sin(finalAngle) * 1;
            finalY = Math.cos(finalAngle) * 1;

            if(x < .05 && x > -.05){
                finalX = 0;
            }
            if(y < .05 && y > -.05){
                finalY = 0;
            }

            robot.MotorLL.setPower(finalX + z);//x + z);
            robot.MotorLR.setPower(finalX + z);//x + z);
            robot.MotorRL.setPower(finalY + z);//y + z);
            robot.MotorRR.setPower(finalY + z);//y + z);







            telemetry.addData("Gyro", getHeading());
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

    public double getHeading() {
        robotHeading = (double)gyro.getIntegratedZValue();
        return robotHeading;
    }


}
