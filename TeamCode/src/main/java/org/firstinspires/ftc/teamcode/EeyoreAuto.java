package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.CameraProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.teamcode.EeyoreHardware.COUNTS_PER_INCH;

@Autonomous(name="Buttered Cardboard (Ball Score)", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
public class EeyoreAuto extends CameraProcessor {
    EeyoreHardware robot = new EeyoreHardware();

    String teamColor = "NONE"; //Initialized as NONE because I don't want color and teamColor to be equal initially
    boolean delay = false;
    int programSelection = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        telemetry.addData("Status:", "Initializing");
        telemetry.update();

        startCamera();

        // Calibrate gyro
        robot.gyro.calibrate();

        while(robot.gyro.isCalibrating()) {
            sleep(100);
        }

        telemetry.addData("Finished Initializing:", robot.gyro.getIntegratedZValue());
        telemetry.update();

        while(!gamepad1.a) {
            if(gamepad1.b) {
                teamColor = "RED";
            } else if(gamepad1.x) {
                teamColor = "BLUE";
            }

            telemetry.addData("Team Color:", teamColor);
            telemetry.update();

            idle();
        }

        telemetry.addData("Status:", "Finished Color Selection. Final Color: " + teamColor);
        telemetry.update();

        waitForStart(); // Wait for the game to start (driver presses PLAY)

        telemetry.addData("Status:", "Moving...");
        telemetry.update();

        turnTester();

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
        gyroTurnSimple(90);
        Thread.sleep(10000);
        gyroTurnSimple(-90);
        Thread.sleep(10000);
        gyroTurnSimple(-180);
        Thread.sleep(3000);
        gyroTurnSimple(180);
        Thread.sleep(3000);
        gyroTurnSimple(-180);
        Thread.sleep(3000);
        gyroTurnSimple(360);
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

    public void turnRobot(int speed, int time)
    {
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - startTime < time)
        {
            robot.l1.setPower(speed);
            robot.l2.setPower(speed);
            robot.r1.setPower(-speed);
            robot.r2.setPower(-speed);
        }
        robot.l1.setPower(0);
        robot.l2.setPower(0);
        robot.r1.setPower(0);
        robot.r2.setPower(0);
    }

    public void gyroTurnNew(int target) throws InterruptedException {
        robot.l1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.r1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idle();

        double Kp = 0.8125; // 0.8125
        double Ki = 0.004; // 0.006
        double Kd = 0.5; // 0.5

        double integral = 0;
        double heading;
        double error_prior = 0;

        robot.gyro.resetZAxisIntegrator();

        while(opModeIsActive()) {
            heading = robot.gyro.getIntegratedZValue();

            double error = (target - heading) / 360;
            integral = integral + error;
            double derivative = error - error_prior;

            double output = 0.8 * Range.clip((Kp * error + Ki * integral + Kd * derivative), -1, 1);

            robot.l1.setPower(-output);
            robot.l2.setPower(-output);
            robot.r1.setPower(output);
            robot.r2.setPower(output);

            if(Math.abs(target - heading) <= 1) {
                break;
            }

            telemetry.addData("Power:", output);
            telemetry.addData("Error:", error);
            telemetry.addData("Kp Pwr:", Kp * error);
            telemetry.addData("Ki Pwr:", Ki * integral);
            telemetry.addData("Kd Pwr:", Kd * derivative);
            telemetry.addData("Heading / Target:", heading + " / " + target);
            telemetry.update();

            idle();
        }

        setDrivePower(0);
    }

    public void gyroTurnSimple(int target) throws InterruptedException {
        robot.l1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.r1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idle();

        int heading = 0;
        int error = 0;
        double output = 0;

        robot.gyro.resetZAxisIntegrator();

        while(opModeIsActive()) {
            heading = robot.gyro.getIntegratedZValue();
            error = target - heading;
            output = 0.3;

            if(Math.abs(error) < 10) {
                output = 0.2;
            }

            if(target < 0) {
                output *= -1;
            }

            robot.l1.setPower(-output);
            robot.l2.setPower(-output);
            robot.r1.setPower(output);
            robot.r2.setPower(output);

            if(Math.abs(error) <= 1) {
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

    public void driveToLine(double power) throws InterruptedException {
        while(robot.color.alpha() == 0) {
            setDrivePower(power);
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
        } else {
            return "RIGHT";
        }
    }

    public String getBeaconColor() {
        while(!imageReady()) {
            telemetry.addData("Camera:", "Waiting for image...");
            telemetry.update();
        }

        Bitmap image = convertYuvImageToRgb(yuvImage, size.width, size.height, 1);

        int red_intensity = 0;
        int blue_intensity = 0;

        for(int x = image.getWidth() / 2; x < image.getWidth(); x++) {
            for(int y = 0; y < image.getHeight(); y++) {
                int pixel = image.getPixel(x, y);

                if(red(pixel) > blue(pixel)) {
                    red_intensity += red(pixel);
                } else {
                    blue_intensity += blue(pixel);
                }
            }
        }

        String left;

        if(red_intensity == 0) { // Homemade exception handling (!)
            red_intensity += 1;
        }

        if(blue_intensity / red_intensity > 100) {
            left = "BLUE";
        } else {
            left = "RED";
        }

        return left;
    }

    public double getWallDistance() {
        return robot.range1.getDistance(DistanceUnit.CM);
    }


    public void determineProgram() throws InterruptedException
    {

        while(!gamepad1.a) {
            if(gamepad1.b) {
                teamColor = "RED";
            } else if(gamepad1.x) {
                teamColor = "BLUE";
            }

            telemetry.addData("Team Color:", teamColor);
            telemetry.update();

            idle();
        }
        telemetry.addData("Status:", "Finished Color Selection. Final Color: " + teamColor);
        telemetry.update();

        Thread.sleep(2000);

        while(!gamepad1.a)
        {
            if(gamepad1.b) {
                delay = true;
            } else if(gamepad1.x) {
                delay = false;
            }

            telemetry.addData("Delay:", delay);
            telemetry.update();

            idle();
        }
        telemetry.addData("Final Delay Selection:", delay);
        telemetry.update();

        Thread.sleep(2000);

        while(!gamepad1.a)
        {
            if(gamepad1.b) {
                programSelection = 0; //0 is flat against the wall, near the ramp
            } else if(gamepad1.x) {
                programSelection = 1; //1 is at an angle on the left side, still pointed at the center goal
            }

            telemetry.addData("Selection:", programSelection);
            telemetry.update();

            idle();
        }

        telemetry.addData("Status:", "Initialized (waiting for start)");
        telemetry.addData("Delay:", delay);
        telemetry.addData("Team Color:", teamColor);
        telemetry.update();
    }

    public void dislodgeBall(String direction) throws InterruptedException
    {
        if(direction.equals("left"))
        {
            int headingStart = robot.gyro.getIntegratedZValue();
            gyroTurnNew(headingStart - 90);
            gyroTurnNew(headingStart + 90);
        } else if(direction.equals("right"))
        {
            int headingStart = robot.gyro.getIntegratedZValue();
            gyroTurnNew(headingStart + 90);
            gyroTurnNew(headingStart - 90);
        }
    }

    public void fireTwice() throws InterruptedException
    {
        robot.shooter1.setPower(1);
        robot.shooter2.setPower(1);

        Thread.sleep(750);

        robot.shooter2.setPower(0);
        robot.shooter1.setPower(0);

        Thread.sleep(1000);

        robot.shooter2.setPower(1);
        robot.shooter1.setPower(1);

        Thread.sleep(750);

        robot.shooter2.setPower(0);
        robot.shooter1.setPower(0);

    }
}