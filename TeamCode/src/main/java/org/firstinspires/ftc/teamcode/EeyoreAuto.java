package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.provider.Settings;

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

        robot.gyro.calibrate();

        while(robot.gyro.isCalibrating()) {
            Thread.sleep(100);
        }

        Thread.sleep(1000);

        setCameraDownsampling(9);
        startCamera();

        telemetry.addData("Status:", "Initialized (waiting for start)");
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

        telemetry.addData("Status:", "Finished Color Selection: " + teamColor);
        telemetry.update();

        waitForStart(); // Wait for the game to start (driver presses PLAY)

        telemetry.addData("Status:", "Moving...");
        telemetry.update();

        // Move off of the wall to within range of the center goal
        moveStraight(16, 0.25);

        // Wait for the robot to settle/stop moving before shooting
        Thread.sleep(1000);

        // Score the pre-loaded ball(s)
        shootShooter(1);
        Thread.sleep(800);
        shootShooter(0);
        Thread.sleep(1000);
        shootShooter(1);
        Thread.sleep(1000);
        shootShooter(0);

        // Turn half-way toward the beacon-wall, parallel to the corner-goal
        gyroTurn(-45);

        // Move straight, parallel to the corner-goal
        moveStraight(55, 0.25);

        // Turn perpendicular to the beacon-wall
        gyroTurn(-90);

        // Move closer to the beacon
        moveStraight(4, 0.25);

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

    public void pressBeaconButton() throws InterruptedException {
        String currentBeaconColor = "NULL";
        do {
            moveStraight(15, 0.3);
            Thread.sleep(100);
            moveStraightSimple(-0.3, 600);
            Thread.sleep(100);
            currentBeaconColor = getBeaconColor();
            telemetry.addData("Beacon Color: ", currentBeaconColor);
            telemetry.update();

        } while(!teamColor.equals(currentBeaconColor));
    }

    public void turn(int speed, int time) {
        long startTime = System.currentTimeMillis();

        while(System.currentTimeMillis() - startTime < time) {
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
                output = 0.10;
            } else {
                output = 0.15;
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

    public void moveStraightSimple(double power, double milliseconds) {
        double startTime = System.currentTimeMillis();

        while(System.currentTimeMillis() - startTime < milliseconds) {
            setDrivePower(power);
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
    public void moveBackwards(double inches, double power) throws InterruptedException {
        robot.l1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.r1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.l1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.r1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        idle();

        int targetLeft = robot.l1.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        int targetRight = robot.r1.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);

        setDrivePower(-power);

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

    public String getBeaconColor() throws InterruptedException {
        while(!imageReady()) {
            telemetry.addData("Camera:", "Waiting for image...");
            telemetry.update();
        }

        long startTime = System.currentTimeMillis();

        Bitmap image = convertYuvImageToRgb(yuvImage, size.width, size.height, 1);

        int red_intensity = 0;
        int blue_intensity = 0;

        for(int x = 0; x < image.getWidth(); x++) {
            for(int y = 0; y < image.getHeight(); y++) {
                int pixel = image.getPixel(x, y);

                if(red(pixel) > blue(pixel)) {
                    red_intensity += red(pixel);
                } else {
                    blue_intensity += blue(pixel);
                }
            }
        }

        String beaconColor;

        if(blue_intensity > red_intensity) {
            beaconColor = "BLUE";
        } else {
            beaconColor = "RED";
        }

        long endTime = System.currentTimeMillis();

        telemetry.addData("Camera Time:", endTime - startTime);
        telemetry.update();

        Thread.sleep(4000);

        return beaconColor;
    }
}