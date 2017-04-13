package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.internal.CameraProcessor;

import static org.firstinspires.ftc.teamcode.EeyoreHardware.COUNTS_PER_INCH;

@Autonomous(name="Gyro Turn Tester", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
public class GyroTurnTester extends CameraProcessor {
    EeyoreHardware robot = new EeyoreHardware();

    private double FASTSPEED = 0.15;
    private double SLOWSPEED = 0.10;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        telemetry.addData("Status:", "Initializing...");
        telemetry.update();

        robot.gyro.calibrate();

        while(robot.gyro.isCalibrating()) {
            Thread.sleep(100);
        }

        Thread.sleep(1000);

        telemetry.addData("Status:", "Initialized (waiting for start)");
        telemetry.update();

        waitForStart(); // Wait for the game to start (driver presses PLAY)

        telemetry.addData("Status:", "Moving...");
        telemetry.update();

        gyroTurn(90);
        Thread.sleep(5000);
        gyroTurn(-90);
        Thread.sleep(5000);
        gyroTurn(45);
        Thread.sleep(5000);
        gyroTurn(-45);
        Thread.sleep(5000);
        gyroTurn(15);
        Thread.sleep(5000);
        gyroTurn(-15);
        Thread.sleep(5000);

        telemetry.addData("Status:", "Shutting down...");
        telemetry.update();

        stopCamera();

        // Run until the end of the match (driver presses STOP)
        while(opModeIsActive()) {
            telemetry.addData("Status:", "Idling...");
            telemetry.update();
            idle();
        }
    }

    public void setDrivePower(double power) {
        robot.l1.setPower(power);
        robot.l2.setPower(power);
        robot.r1.setPower(power);
        robot.r2.setPower(power);
    }

    public void gyroTurn(int target) throws InterruptedException {
        robot.l1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.r1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idle();

        double heading = 0;
        double error = 0;
        double output = 0;

        while(opModeIsActive()) {
            heading = robot.gyro.getIntegratedZValue();
            error = target - heading;

            if(Math.abs(error) < 20) {
                output = SLOWSPEED;
            } else {
                output = FASTSPEED;
            }

            if(error < 0) {
                output = -output;
            }

            robot.l1.setPower(-output);
            robot.l2.setPower(-output);
            robot.r1.setPower(output);
            robot.r2.setPower(output);

            if((Math.abs(error) - 2) < 0) {
                break;
            }

            idle();
        }

        setDrivePower(0);

        telemetry.addData("Power:", output);
        telemetry.addData("Error:", error);
        telemetry.addData("Heading / Target:", heading + " / " + target);
        telemetry.update();
    }
}