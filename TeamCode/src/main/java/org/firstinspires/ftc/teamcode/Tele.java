package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp", group = "frame")
public abstract class Tele extends TeleSkele{
    private static final double ACCEPTINPUTTHRESHOLD = 0.15;
    private boolean holding = false;
    private boolean robotCentric = true;
    public void runOpMode() {
        initRobot();
        while(!opModeIsActive()) {
            if (!Gyro.initialized) {
                initTeleGyro();
                OFFSET = 0;
                telemetry.addLine("Gyro not initialized, initializing gyro!");
            }
            if (gamepad1.dpad_up) {
                initTeleGyro();
                OFFSET = 0;
                telemetry.addLine("Init requested, initializing gyro!");
            }
            telemetry.update();
            waitForStart();
        }
        while (opModeIsActive()) {
            telemetry.addData("Holding", holding);
            if(robotCentric){
                telemetry.addLine("Drive System: Robot-Centric");
            }else{
                telemetry.addLine("Drive System: Field-Centric");
                telemetry.addData("Heading", getHeading());
            }
            if(driveDirection == -1){
                telemetry.addLine("Drive Direction: Inverted");
            }else{
                telemetry.addLine("Drive Direction: Direct");
            }
            telemetry.addData("Drive Speed", speed.getSpeed());
            telemetry.update();
            if(gamepad1.x){
                driveDirection = -1;
            }else if(gamepad1.y){
                driveDirection = 1;
            }
            if(gamepad1.left_trigger > 0.1){
                speed = DriveSpeed.HIGHPOWER;
            }else if(gamepad1.right_trigger > 0.1){
                speed = DriveSpeed.LOWPOWER;
            }else{
                speed = DriveSpeed.MIDPOWER;
            }
            double inputX, inputY, inputC;
            inputX = Math.abs(gamepad1.left_stick_x) > ACCEPTINPUTTHRESHOLD ? -gamepad1.left_stick_x : 0;
            inputY = Math.abs(gamepad1.left_stick_y) > ACCEPTINPUTTHRESHOLD ? gamepad1.left_stick_y : 0;
            inputC = Math.abs(gamepad1.right_stick_x) > ACCEPTINPUTTHRESHOLD ? -gamepad1.right_stick_x : 0;
            if(robotCentric) {
                arcadeMecanum(-inputY, inputX, -inputC);
            }else{
                fieldCentric(inputY, inputX, -inputC);
            }


            if (gamepad1.dpad_left){
                OFFSET = 0;
                initTeleGyro();
            }
            while (gamepad1.a && gamepad2.a) {
                setPower(0);
            }
            //Motors
            if(gamepad2.a){
                motor1.setPower(1);
            } else {
                motor1.setPower(0);
            }
            if(gamepad2.b){
                motor2.setPower(1);
            } else {
                motor2.setPower(0);
            }
            if(gamepad2.x){
                motor3.setPower(1);
            } else {
                motor3.setPower(0);
            }
            if(gamepad2.y){
                motor4.setPower(1);
            } else {
                motor4.setPower(0);
            }
            //Cr Servo
            if(gamepad2.right_trigger > .1){
                crservo1.setPower(gamepad2.right_trigger);
            } else {
                crservo1.setPower(-gamepad2.right_trigger);
            }
            //Servo
            if(gamepad2.left_bumper){
                servo1.setPosition(.5);
            } else {
                servo1.setPosition(0);
            }
        }
    }
}

