package org.firstinspires.ftc.teamcode.ITDBHardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class AllHardware {
    private OpMode myOpMode;

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;
    /*
    public DcMotorEx linear;
    public DcMotorEx intake;
    public DcMotorEx rotation;

    public ServoImplEx claw;
    public ServoImplEx swivel;
*/
    public IMU imu;

    private HardwareMap hwMap;
    public AllHardware(OpMode opmode) {
        myOpMode = opmode;
        hwMap = myOpMode.hardwareMap;
    }

    public void init()    {
        frontLeft = hwMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hwMap.get(DcMotorEx.class, "frontRight");
        backLeft = hwMap.get(DcMotorEx.class, "backLeft");
        backRight = hwMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
/*
        claw = hwMap.get(ServoImplEx.class, "claw");
        swivel = hwMap.get(ServoImplEx.class, "swivel");

        rotation = hwMap.get(DcMotorEx.class, "rotation");
        rotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotation.setDirection(DcMotor.Direction.FORWARD);
        rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linear = hwMap.get(DcMotorEx.class, "linear");
        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
*/
        imu = hwMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();

        //intake = hwMap.get(DcMotorEx.class, "intake");
    }
}