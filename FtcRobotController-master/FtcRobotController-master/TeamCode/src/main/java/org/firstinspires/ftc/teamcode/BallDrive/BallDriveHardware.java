package org.firstinspires.ftc.teamcode.BallDrive;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BallDriveHardware {
    // Motors and Servos
    public DcMotorEx MotorLL;
    public DcMotorEx MotorLR;
    public DcMotorEx MotorRL;
    public DcMotorEx MotorRR;
    public CRServo Servo1;
    public CRServo Servo2;



    HardwareMap BDHardware;

    public void init(HardwareMap BDHardware){

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
