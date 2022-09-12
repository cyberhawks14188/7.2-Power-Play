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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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
    double RLDIR,RRDIR,LRDIR,LLDIR;
    double maxmotor, speed;

    @Override

    public void runOpMode(){
        robot.init(hardwareMap);

       // gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        //telemetry.addData(">", "Calibrating Gyro");    //
        //telemetry.update();

        /*gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();


        gyro.resetZAxisIntegrator();
        getHeading();

//test*/
        waitForStart();

        while (opModeIsActive()){
            Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            telemetry.addData("imu", robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
            telemetry.addData("heading", angles.firstAngle);

            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
           // if(gamepad1.right_bumper){
           //     z = gamepad1.right_stick_x * .5;
           // }else{
                z = gamepad1.right_stick_x;
           // }


            desiredHeading = Math.toDegrees(Math.atan2(x,y));


            if(x == 0){
                if(y<0){
                    desiredHeading = 180;
                }else{
                    desiredHeading = 0;
                }
            }

            finalAngle = desiredHeading;// + angles.firstAngle;
            finalX = Math.sin(finalAngle) * 1;
            finalY = Math.cos(finalAngle) * 1;

            if(x < .05 && x > -.05){
                finalX = 0;
            }
            if(y < .05 && y > -.05){
                finalY = 0;
            }

            //robot.MotorLL.setPower(.25*(finalY + z));//x + z);
            //robot.MotorLR.setPower(.25*(-finalY + z));//x + z);
            //robot.MotorRL.setPower(.25*(finalX + z));//y + z);
            //robot.MotorRR.setPower(.25*(-finalX + z));//y + z);
            LLDIR = finalY + z;
            LRDIR = -finalY + z;
            RLDIR = -finalX + z;
            RRDIR = finalX + z;

            maxmotor = Math.max(Math.max(Math.abs(LLDIR), Math.abs(LRDIR)), Math.max(Math.abs(RLDIR), Math.abs(RRDIR)));

            LLDIR = LLDIR/maxmotor;
            LRDIR = LRDIR/maxmotor;
            RLDIR = RLDIR/maxmotor;
            RRDIR = RRDIR/maxmotor;

            speed = Math.hypot(x, y) + Math.abs(z);
            if(speed >= 1){
                speed = 1;
            }
            if(gamepad1.right_bumper){
                speed = .6;
            }


            robot.MotorLL.setPower(LLDIR*speed);
            robot.MotorLR.setPower(LRDIR*speed);
            robot.MotorRL.setPower(RLDIR*speed);
            robot.MotorRR.setPower(RRDIR*speed);
            //robot.MotorLL.setPower(y + z);
            //robot.MotorLR.setPower(-y + z);
            //robot.MotorRL.setPower(-x + z);
            //robot.MotorRR.setPower(x + z);








            //telemetry.addData("Gyro", getHeading());
            telemetry.addData("final angle", finalAngle);



            telemetry.addData("LL Current", robot.MotorLL.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("LR Current", robot.MotorLR.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("RL Current", robot.MotorRL.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("RR Current", robot.MotorRR.getCurrent(CurrentUnit.MILLIAMPS));
            dashboardTelemetry.addData("LL Current", robot.MotorLL.getCurrent(CurrentUnit.MILLIAMPS));
            dashboardTelemetry.addData("LR Current", robot.MotorLR.getCurrent(CurrentUnit.MILLIAMPS));
            dashboardTelemetry.addData("RL Current", robot.MotorRL.getCurrent(CurrentUnit.MILLIAMPS));
            dashboardTelemetry.addData("RR Current", robot.MotorRR.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("y",y);
            telemetry.addData("x",x);
            telemetry.addData("z",z);
            telemetry.addData("FinalX", finalX);
            telemetry.addData("FinalY", finalY);
            dashboardTelemetry.update();

            telemetry.update();

        }

    }

    /*public double getHeading() {
        robotHeading = (double)gyro.getIntegratedZValue();
        return robotHeading;
    }*/


}
