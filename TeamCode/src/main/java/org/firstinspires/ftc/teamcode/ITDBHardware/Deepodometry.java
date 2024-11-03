package org.firstinspires.ftc.teamcode.ITDBHardware;


import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name= "Deepodometry")


public class Deepodometry extends LinearOpMode {
    AllHardware robot;

    @Override
    public void runOpMode(){

        robot = new AllHardware(this);

        robot.init();

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.myOtos.resetTracking();
        robot.myOtos.calibrateImu();
        waitForStart();
        SparkFunOTOS.Pose2D pos;
        pos = robot.myOtos.getPosition();
        //LinearMovement(5, 0, 0, .6);
        telemetry.addLine("first thing");
        telemetry.addLine("X Position :" +pos.x);
        telemetry.addLine("Y Position :" +pos.y);
        telemetry.update();
        // SCORING THE PRELOADED SPECIMEN FIRST

        LinearMovement(36, 22,0,1);
        sleep(1000);

        sleep(5000);

        sleep(2000);

        sleep(1000);
        // GOING TO GRAB ANOTHER NUETRAL BLOCK AND SCORE IT
        LinearMovement(2, 22,0,1);
        //sleep(1000);
        LinearMovement(-12,12,135,1);
        //sleep(1000);
        //LinearMovement(0, 0,0,1);
        //sleep(1000);
        pos = robot.myOtos.getPosition();
        telemetry.addLine("first thing");
        telemetry.addLine("X Position :" +pos.x);
        telemetry.addLine("Y Position :" +pos.y);
        telemetry.update();
        //LinearMovement(-5, 3,0,.6);
        pos = robot.myOtos.getPosition();
        telemetry.addLine("X Position :" +pos.x);
        telemetry.addLine("Y Position :" +pos.y);
        telemetry.update();
        sleep(2000);
    }


    public void LinearMovement(double Xpos, double Ypos, double Heading, double Speed){
        while (opModeIsActive()) {
            SparkFunOTOS.Pose2D pos = robot.myOtos.getPosition();
            double strafe = Xpos - pos.x;
            double forward = Ypos - pos.y;
            double turn = Heading - pos.h;

            forward *= -1;
            strafe *= -1;



            if (Math.abs(strafe) < 0.25 && Math.abs(forward) < 0.25) {
                robot.drivePower(0, 0, 0);
                break;
            }
            double P = 0.06;
            strafe *= P;
            forward *= P;
            turn *= 0.04;

            strafe = Range.clip(strafe, -Speed, Speed);
            forward = Range.clip(forward, -Speed, Speed);

            double botHeading = Math.toRadians(pos.h);

            double rotStrafe = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
            double rotForward = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);



            telemetry.addData("Robot x pos wanted", Xpos);
            telemetry.addData("Robot y pos wanted", Ypos);

            telemetry.addData("Robot x pos current", pos.x);
            telemetry.addData("Robot y pos current", pos.y);


            telemetry.addData("Robot strafe", strafe);
            telemetry.addData("Robot move forward", forward);

            telemetry.update();

            robot.drivePower(rotForward, rotStrafe, turn);

        }
    }
}