package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Gyro Test", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class GyroTest extends LinearOpMode {
    public BNO055IMU gyro = null;

    @Override
    public void runOpMode() throws InterruptedException {
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyro.initialize(parameters);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while(opModeIsActive()) {
            Position position = gyro.getPosition();

            telemetry.addData("Status", "Running...");
            telemetry.addData("Roll", "%.5f", normalizeAngle(getGyroRoll()));
            telemetry.addData("Pitch", "%.5f", normalizeAngle(getGyroPitch()));
            telemetry.addData("Yaw", "%.5f", normalizeAngle(getGyroYaw()));
            telemetry.addData("Position", position.x + "/" + position.y + "/" + position.z);
            telemetry.update();

            idle();
        }
    }

    public double getGyroRoll() {
        Quaternion angles = gyro.getQuaternionOrientation();

        double w = angles.w;
        double x = angles.x;
        double y = angles.y;
        double z = angles.z;

        return Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * 180.0 / Math.PI;
    }

    public double getGyroPitch() {
        Quaternion angles = gyro.getQuaternionOrientation();

        double w = angles.w;
        double x = angles.x;
        double y = angles.y;
        double z = angles.z;

        return Math.asin(2 * (w * y - x * z)) * 180.0 / Math.PI;
    }

    public double getGyroYaw() {
        Quaternion angles = gyro.getQuaternionOrientation();

        double w = angles.w;
        double x = angles.x;
        double y = angles.y;
        double z = angles.z;

        return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * 180.0 / Math.PI;
    }

    public double normalizeAngle(double angle) {
        return angle - (angle > 0 ? 360 : 0);
    }
}