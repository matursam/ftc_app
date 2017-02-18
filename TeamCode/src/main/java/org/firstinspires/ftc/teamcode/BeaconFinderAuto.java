package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.CameraProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.teamcode.EeyoreHardware.COUNTS_PER_INCH;

@Autonomous(name="Beacon Finder", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
public class BeaconFinderAuto extends CameraProcessor {
    EeyoreHardware robot = new EeyoreHardware();

    String teamColor = "NONE"; //Initialized as NONE because I don't want color and teamColor to be equal initially
    String returnedSide;

    class BeaconColors {
        String left = null;
        String right = null;

        BeaconColors(String left, String right) {
            this.left = left;
            this.right = right;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        telemetry.addData("Status:", "Initializing");
        telemetry.update();

        // Calibrate gyro
        robot.gyro.calibrate();

        while(robot.gyro.isCalibrating()) {
            Thread.sleep(50);
            idle();
        }

        Thread.sleep(2000);

        // Initiate camera
        setCameraDownsampling(9);
        startCamera();

        while(!gamepad1.a && (teamColor == "NONE")) {
            if(gamepad1.b) {
                teamColor = "RED";
            } else if(gamepad1.x) {
                teamColor = "BLUE";
            }

            telemetry.addData("Team Color:", teamColor);
            telemetry.update();

            idle();
        }

        telemetry.addData("Status:", "Initialized (waiting for start)");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Status:", "Moving...");
        telemetry.update();

        // Move off of the wall
        moveStraight(14, 0.4);
        Thread.sleep(1000);
        shootShooter(1);
        Thread.sleep(500);
        shootShooter(1);
        Thread.sleep(500);
        moveStraight(6, 0.4);

        // Now we move on to the beacons
        if(teamColor == "RED") { //We are red
            gyroTurn(45);
            moveStraight(18, 0.4);
            gyroTurn(90);
            moveStraight(31, 0.4);
            Thread.sleep(500);
            gyroTurn(180);
            Thread.sleep(500);

            robot.l1.setDirection(DcMotor.Direction.FORWARD);
            robot.l2.setDirection(DcMotor.Direction.FORWARD);
            robot.r1.setDirection(DcMotor.Direction.REVERSE);
            robot.r2.setDirection(DcMotor.Direction.REVERSE);
        } else { //We are blue
            gyroTurn(-45);
            moveStraight(18, 0.4);
            gyroTurn(-90);
            moveStraight(31, 0.4);
            Thread.sleep(500);
            gyroTurn(0);
            Thread.sleep(500);
        }

        // Press the beacons
        gyroTurn(0);
        driveToLine();
        Thread.sleep(5000);

        String beacon1 = getBeaconSide(25);

        if(beacon1 == "LEFT") {
            pressLeftButton();
        } else if(beacon1 == "RIGHT") {
            pressRightButton();
        }

        // Move partly to the second beacon
        moveStraight(40, 0.6);

        driveToLine();
        Thread.sleep(2000);

        String beacon2 = getBeaconSide(25);

        if(beacon2 == "LEFT") {
            pressLeftButton();
        } else if(beacon2 == "RIGHT") {
            pressRightButton();
        }

        telemetry.addData("Status:", "Shutting down...");
        telemetry.update();

        stopCamera();

        // Run until the end of the match (driver presses STOP)
        while(opModeIsActive()) {
            telemetry.addData("Status:", "Idling...");
            telemetry.addData("Distance: ", getWallDistance());
            telemetry.update();
            idle();
        }
    }

    public void shootShooter(int power) throws InterruptedException {
        robot.shooter1.setPower(power);
        robot.shooter2.setPower(power);
        Thread.sleep(750);
        robot.shooter1.setPower(0);
        robot.shooter2.setPower(0);
    }

    public void setDrivePower(double power) {
        robot.l1.setPower(power);
        robot.l2.setPower(power);
        robot.r1.setPower(power);
        robot.r2.setPower(power);
    }

    public void driveLeft(double power) {
        robot.l1.setPower(-power);
        robot.l2.setPower(-power);
        robot.r1.setPower(power);
        robot.r2.setPower(power);
    }

    public void driveRight(double power) {
        robot.l1.setPower(power);
        robot.l2.setPower(power);
        robot.r1.setPower(-power);
        robot.r2.setPower(-power);
    }

    public void pressLeftButton() throws InterruptedException {
        robot.leftPresser.setPosition(0.2);
        Thread.sleep(5000);
        robot.leftPresser.setPosition(0.8);
    }

    public void pressRightButton() throws InterruptedException {
        robot.rightPresser.setPosition(0.2);
        Thread.sleep(5000);
        robot.rightPresser.setPosition(0.8);
    }

    public void turnTester() throws InterruptedException {
        gyroTurn(90);
        Thread.sleep(3000);
        gyroTurn(45);
        Thread.sleep(3000);
        gyroTurn(0);
        Thread.sleep(3000);
        gyroTurn(-45);
        Thread.sleep(3000);
        gyroTurn(-90);
        Thread.sleep(3000);
        gyroTurn(-180);
        Thread.sleep(3000);
        gyroTurn(0);
    }

    public void gyroTurn(int target) throws InterruptedException {
        robot.l1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.r1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idle();

        double Kp = 0.8125;//0.8125
        double Ki = 0.004;//0.006
        double Kd = 0.5;//0.5
        double integral = 0;
        double heading;
        double buffer = 4;
        double error_prior = 0;

        int successfulLoops = 0;

        while(successfulLoops < 5000) {
            heading = robot.gyro.getIntegratedZValue();
            double error = (target - heading) / 360;

            integral = integral + error;

            if(integral > 75) {
                integral = 75;
            } else if(integral < -75) {
                integral = -75;
            }

            if(integral < 60 && integral > 0) {
                integral = 60;
            } else if(integral > -60 && integral < 0) {
                integral = -60;
            }

            double derivative = error - error_prior;

            if(error < 0 && integral > 0) {
                integral = 0;
            }

            if(error > 0 && integral < 0) {
                integral = 0;
            }

            double output = 0.8 * Range.clip((Kp * error + Ki * integral + Kd * derivative), -1, 1);

            if(Math.abs(heading - target) < buffer) {
                successfulLoops += 1;
                integral = 0;
                output = 0;
            } else {
                successfulLoops = 0;
            }

            robot.l1.setPower(-output);
            robot.l2.setPower(-output);
            robot.r1.setPower(output);
            robot.r2.setPower(output);

            telemetry.addData("Power:", output);
            telemetry.addData("Error:", error);
            telemetry.addData("Kp Pwr:", Kp * error);
            telemetry.addData("Ki Pwr:", Ki * integral);
            telemetry.addData("Kd Pwr:", Kd * derivative);
            telemetry.addData("Heading / Target:", heading + " / " + target);
            telemetry.update();

            if(!opModeIsActive()) {
                break;
            }

            if(Math.abs(heading - target) <= buffer) {
                successfulLoops += 1;
                integral = 0;
            } else {
                successfulLoops = 0;
            }

            idle();
        }

        setDrivePower(0);
    }

    public void moveStraight(double inches, double power) throws InterruptedException {
        robot.l1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.r1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.l1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.r1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        idle();

        int targetLeft = robot.l1.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int targetRight = robot.r1.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

        setDrivePower(power);

        while((robot.l1.getCurrentPosition() < targetLeft) && (robot.r1.getCurrentPosition() < targetRight)) {
            telemetry.addData("Target", "left=" + targetLeft + " " + "right=" + targetRight);
            telemetry.addData("Position", "left=" + robot.l1.getCurrentPosition() + " " + "right=" + robot.r1.getCurrentPosition());
            telemetry.addData("Power", "left=" + robot.l1.getPower() + " " + "right=" + robot.r1.getPower());
            telemetry.update();

            idle();
        }

        setDrivePower(0);
    }

    public void driveToLine() throws InterruptedException {
        while(robot.color.alpha() == 0) {
            setDrivePower(0.1);
            Thread.sleep(50);
            setDrivePower(0);
            Thread.sleep(100);
        }

        setDrivePower(0);
    }

    public String getBeaconSide(int passes) throws InterruptedException {
        int leftRed = 0;
        int leftBlue = 0;

        for(int i = 0; i < passes; i++) {
            String leftColor = getBeaconColor();

            if(leftColor == "BLUE") {
                leftBlue++;
            } else if(leftColor == "RED") {
                leftRed++;
            }
        }

        String left;

        if(leftRed > leftBlue) {
            left = "RED";
        } else if(leftBlue > leftRed) {
            left = "BLUE";
        } else {
            return "ERROR";
        }

        if(left == teamColor) {
            return "LEFT";
        } else if(left != teamColor) {
            return "RIGHT";
        }

        return "ERROR";
    }

    public String getBeaconColor() {
        while(!imageReady()) {
            telemetry.addData("Camera:", "Waiting for image...");
            telemetry.update();
        }

        Bitmap image = convertYuvImageToRgb(yuvImage, size.width, size.height, 1);

        int left_intensity = 0;

        for(int x = 0; x < image.getWidth() / 2; x++) {
            for(int y = 0; y < image.getHeight(); y++) {
                int pixel = image.getPixel(x, y);
                int pixel_blue = blue(pixel);

                left_intensity += pixel_blue;
            }
        }

        int right_intensity = 0;

        for(int x = image.getWidth() / 2; x < image.getWidth(); x++) {
            for(int y = 0; y < image.getHeight(); y++) {
                int pixel = image.getPixel(x, y);
                int pixel_blue = blue(pixel);

                right_intensity += pixel_blue;
            }
        }

        String left;

        if(left_intensity < right_intensity) {
            left = "BLUE";
        } else if(right_intensity > left_intensity) {
            left = "RED";
        } else {
            left = "ERROR";
        }

        return left;
    }

    public double getWallDistance() {
        return (robot.range1.getDistance(DistanceUnit.CM) + robot.range2.getDistance(DistanceUnit.CM)) / 2;
    }
}