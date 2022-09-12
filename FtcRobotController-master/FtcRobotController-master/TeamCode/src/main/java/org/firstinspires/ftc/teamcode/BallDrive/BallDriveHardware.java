package org.firstinspires.ftc.teamcode.BallDrive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class BallDriveHardware {
    // Motors and Servos
    public DcMotorEx MotorLL;
    public DcMotorEx MotorLR;
    public DcMotorEx MotorRL;
    public DcMotorEx MotorRR;
    public CRServo Servo1;
    public CRServo Servo2;
    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;






    HardwareMap BDHardware;

    public void init(HardwareMap BDHardware){

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = BDHardware.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



        // Define motors and servos
        MotorLL = BDHardware.get(DcMotorEx.class, "Motor1");
        MotorLR = BDHardware.get(DcMotorEx.class, "Motor2");
        MotorRL = BDHardware.get(DcMotorEx.class, "Motor3");
        MotorRR = BDHardware.get(DcMotorEx.class, "Motor4");
        Servo1 = BDHardware.get(CRServo.class, "CRServo1");
        Servo2 = BDHardware.get(CRServo.class, "CRServo2");


        MotorLL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorRL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorLL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorLR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorRL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorLL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorRL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
           }

}
