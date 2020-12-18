package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.Arrays;

public abstract class TeleSkele extends LinearOpMode {
    public DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, motor1, motor2, motor3, motor4;
    CRServo crservo1;
    Servo servo1;
    double startingHeading;
    public static int OFFSET = -90;
    private final double SCALAR = 1 / .7071067811865475;
    DriveSpeed speed = DriveSpeed.LOWPOWER;
    int driveDirection = 1;
    public Orientation angles;
    //Nevrest 20s have 560 TPR, 40 uses 1120, 60 uses 1680
    //28 * 20 / (2ppi * 4.125)

    //this method is the base drive and mathematics for our mecanum algorithmic drive
    public void arcadeMecanum(double y, double x, double c) {

        double leftFrontVal = (y * driveDirection) - (x * driveDirection) + c;
        double rightFrontVal = -(y * driveDirection) - (x * driveDirection) + c;

        // try setting these equal to their front most counterpart
        double leftBackVal = (y * driveDirection) + (x * driveDirection) + c;
        double rightBackVal = -(y * driveDirection) + (x * driveDirection) + c;

        //Move range to between 0 and +1, if not already
        double[] wheelPowers = {rightFrontVal, leftFrontVal, leftBackVal, rightBackVal};
        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            leftFrontVal /= wheelPowers[3];
            rightFrontVal /= wheelPowers[3];
            leftBackVal /= wheelPowers[3];
            rightBackVal /= wheelPowers[3];
        }
        //double scaledPower = gearShift[gear];
        leftFrontWheel.setPower(leftFrontVal * speed.getSpeed());
        rightFrontWheel.setPower(rightFrontVal * speed.getSpeed());
        leftBackWheel.setPower(leftBackVal * speed.getSpeed());
        rightBackWheel.setPower(rightBackVal * speed.getSpeed());
    }

    private void arcadeMecanumTrig(double y, double x, double c) {
        //,7071067811865475
        double angle = Math.atan2(y, x);
        double magnitude = Math.sqrt((Math.pow(y, 2) + Math.pow(x, 2)));
        double rightFrontVal = -(driveDirection * speed.getSpeed() * (SCALAR * ((Math.sin(angle + .25 * Math.PI) * magnitude) + c)));
        double leftBackVal = (driveDirection * speed.getSpeed() * (SCALAR * (Math.sin(angle + .25 * Math.PI) * magnitude) - c));
        double leftFrontVal = (driveDirection * speed.getSpeed() * (SCALAR * (Math.sin(angle - .25 * Math.PI) * magnitude) - c));
        double rightBackVal = -(driveDirection * speed.getSpeed() * (SCALAR * (Math.sin(angle - .25 * Math.PI) * magnitude) + c));
        double[] wheelPowers = {rightFrontVal, leftFrontVal, leftBackVal, rightBackVal};
        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            leftFrontVal /= wheelPowers[3];
            rightFrontVal /= wheelPowers[3];
            leftBackVal /= wheelPowers[3];
            rightBackVal /= wheelPowers[3];
        }
        leftFrontWheel.setPower(leftFrontVal);
        rightFrontWheel.setPower(rightFrontVal);
        leftBackWheel.setPower(leftBackVal);
        rightBackWheel.setPower(rightBackVal);
    }
//
    //Translates x and y values through the gyro for field-centric drive before passing them to arcadeMecanum
    public void fieldCentric(double y, double x, double c) {
        double forward = -y;
        double strafe = x;
        angles = Gyro.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double gyroDegrees = -angles.firstAngle + OFFSET;//make this negative
        double gyroRadians = gyroDegrees * (Math.PI / 180);
        double temp = forward * Math.cos(gyroRadians) + strafe * Math.sin(gyroRadians);
        strafe = (-forward * Math.sin(gyroRadians)) + (strafe * Math.cos(gyroRadians));
        forward = temp;
        arcadeMecanum(forward, strafe, c);
    }

    public void initRobot() {
        //Mechanum wheels
        leftFrontWheel = hardwareMap.dcMotor.get("left_front");
        leftBackWheel = hardwareMap.dcMotor.get("left_back");
        rightFrontWheel = hardwareMap.dcMotor.get("right_front");
        rightBackWheel = hardwareMap.dcMotor.get("right_back");
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    double getHeading() {
        angles = Gyro.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        startingHeading = -angles.firstAngle;//make this negative
        return startingHeading;
    }

    /*
    This function is called at the beginning of the program to activate
    the IMU Integrated Gyro.
     */
    public void initTeleGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //
        Gyro.imu = hardwareMap.get(BNO055IMU.class, "imu");
        Gyro.imu.initialize(parameters);
        Gyro.initialized = true;
    }
//
    void setMode(DcMotor.RunMode x) {
        leftFrontWheel.setMode(x);
        rightFrontWheel.setMode(x);
        leftBackWheel.setMode(x);
        rightBackWheel.setMode(x);
    }

    void setPower(double power) {
        leftFrontWheel.setPower(power);
        rightFrontWheel.setPower(power);
        leftBackWheel.setPower(power);
        rightBackWheel.setPower(power);
    }
}

