package org.firstinspires.ftc.teamcode.ITDBHardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
    SparkFunOTOS myOtos;

    private void configureOtos() {

        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);


        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);


        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

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
        myOtos = hwMap.get(SparkFunOTOS.class, "sensor_otos");
        configureOtos();
        //intake = hwMap.get(DcMotorEx.class, "intake");
    }
    public void setDrivePower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);
    }
    public void drivePower(double forward, double turn, double yaw) {
        double leftFrontPower = forward + turn + yaw;
        double rightFrontPower = forward - turn - yaw;
        double leftBackPower = forward - turn + yaw;
        double rightBackPower = forward + turn - yaw;
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));


        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }
}