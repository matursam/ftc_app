package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

public class EeyoreHardware
{
    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 0.75;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    /* Public OpMode members. */
    public DcMotor l1 = null;
    public DcMotor l2 = null;
    public DcMotor r1 = null;
    public DcMotor r2 = null;
    public DcMotor shooter1 = null;
    public DcMotor shooter2 = null;
    public DcMotor collection = null;
    public Servo leftPresser = null;
    public Servo rightPresser = null;
    public ColorSensor color = null;
    public ModernRoboticsI2cGyro gyro = null;
    public ModernRoboticsI2cRangeSensor range1 = null;
    public ModernRoboticsI2cRangeSensor range2 = null;

    /* Local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public EeyoreHardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors and Sensors
        l1 = hwMap.dcMotor.get("l1");
        l2 = hwMap.dcMotor.get("l2");
        r1 = hwMap.dcMotor.get("r1");
        r2 = hwMap.dcMotor.get("r2");
        shooter1 = hwMap.dcMotor.get("shooter1");
        shooter2 = hwMap.dcMotor.get("shooter2");
        collection = hwMap.dcMotor.get("collection");
        leftPresser = hwMap.servo.get("button_left");
        rightPresser = hwMap.servo.get("button_right");
        color = hwMap.colorSensor.get("color");
        gyro = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro");
        range1 = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range1");
        range2 = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range2");

        // Set motor direction
        l1.setDirection(DcMotor.Direction.REVERSE);
        l2.setDirection(DcMotor.Direction.REVERSE);
        r1.setDirection(DcMotor.Direction.FORWARD);
        r2.setDirection(DcMotor.Direction.FORWARD);
        shooter1.setDirection(DcMotor.Direction.FORWARD);
        shooter2.setDirection(DcMotor.Direction.FORWARD);
        collection.setDirection(DcMotor.Direction.FORWARD);

        // Set servo direction
        leftPresser.setDirection(Servo.Direction.REVERSE);
        rightPresser.setDirection(Servo.Direction.REVERSE);

        // Set all motors to zero power
        l1.setPower(0);
        l2.setPower(0);
        r1.setPower(0);
        r2.setPower(0);
        shooter1.setPower(0);
        shooter2.setPower(0);
        collection.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        l1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collection.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize servos
        leftPresser.setPosition(0.8);
        rightPresser.setPosition(0.8);
    }

    public void waitForTick(long periodMs) throws InterruptedException {
        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }


}