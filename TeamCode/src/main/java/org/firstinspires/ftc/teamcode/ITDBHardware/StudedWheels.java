package org.firstinspires.ftc.teamcode.ITDBHardware;




import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVisionColorLocator;

@TeleOp(name = "StuddedDrive", group = "Linear OpMode")
public class StudedWheels extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    AllHardware robot;

    @Override
    public void runOpMode() {
        robot = new AllHardware(this);
        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // set yaw to zero
            if (gamepad1.back) {
                robot.imu.resetYaw();
            }

            double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = - gamepad1.left_stick_x;
            // Rotate the movement direction with the bot rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double max;
            double yaw = gamepad1.right_stick_x;

            double leftFrontPower = rotY + rotX - yaw;
            double rightFrontPower = rotY - rotX + yaw;
            double leftBackPower = rotY - rotX - yaw;
            double rightBackPower = rotY + rotX + yaw;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            max = Math.max(max, 1.5); // remove this line to speed up robot
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            robot.frontLeft.setPower(leftFrontPower);
            robot.frontRight.setPower(rightFrontPower);
            robot.backLeft.setPower(leftBackPower);
            robot.backRight.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            //telemetry.addData("rotation", "Rotation: " + robot.rotation.getCurrentPosition());
            //telemetry.addData("Linear Position", robot.linear.getCurrentPosition());
            //telemetry.addData("Linear Controller position", robot.linear.getController().getMotorCurrentPosition(1));
            //telemetry.addData("swivel", "Swivel position:", robot.swivel.getPosition());
            telemetry.update();
        }
    }
}

