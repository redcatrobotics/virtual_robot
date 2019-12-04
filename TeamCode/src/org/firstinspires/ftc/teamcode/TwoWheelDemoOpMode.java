package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "two wheel demo opmode", group = "TwoWheel")
public class TwoWheelDemoOpMode extends OpMode {

    private DcMotor left = null;
    private DcMotor right = null;
    private GyroSensor gyro = null;
    private Servo backServo = null;
    private ColorSensor colorSensor = null;
    private DistanceSensor frontDistance = null;
    private DistanceSensor leftDistance = null;
    private DistanceSensor backDistance = null;
    private DistanceSensor rightDistance = null;
    private double encoder_start_l;
    private double encoder_start_r;
    boolean halt = false;
    double encoder_change = 0.0;
    double gyro_heading_start = 0.0;
    double gyro_heading_change = 0.0;
    long stage = 1;

    private ElapsedTime et = null;
    private int waitForStartTime = 0;

    public void init(){
        left = hardwareMap.dcMotor.get("left_motor");
        right = hardwareMap.dcMotor.get("right_motor");
        left.setDirection(DcMotor.Direction.REVERSE);
        gyro = hardwareMap.gyroSensor.get("gyro_sensor");
        backServo = hardwareMap.servo.get("back_servo");
        gyro.init();
        colorSensor = hardwareMap.colorSensor.get("color_sensor");
        frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");
        rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");

        et = new ElapsedTime();
        encoder_start_l = left.getCurrentPosition();
        encoder_start_r = right.getCurrentPosition();


    }

    public void init_loop(){
        if (et.milliseconds() >= 1000) {
            waitForStartTime++;
            et.reset();
        }
        telemetry.addData("Press Start to Continue"," %d", waitForStartTime);
    }

    public void loop(){
//        if (gamepad1.a){
//            telemetry.addData("a pressed","");
//            left.setPower(-.5);
//            right.setPower(-.5);
//        } else if (gamepad1.y) {
//            telemetry.addData("y pressed", "");
//            left.setPower(0.5);
//            right.setPower(0.5);
//        } else if (gamepad1.b){
//            telemetry.addData("b pressed", "");
//            left.setPower(0.5);
//            right.setPower(-0.5);
//        } else if (gamepad1.x){
//            telemetry.addData("x pressed", "");
//            left.setPower(-0.5);
//            right.setPower(0.5);
//        } else {
//            left.setPower(0.1);
//            right.setPower(0.05);
//        }
        encoder_change = left.getCurrentPosition() - encoder_start_l;
        gyro_heading_change = gyro_heading_start - gyro.getHeading();
        if (gyro_heading_change < -180){
            gyro_heading_change = gyro_heading_change + 360;
        }
        if (gyro_heading_change > 180){
            gyro_heading_change = gyro_heading_change - 360;
        }



        if (stage == 1 &  encoder_change > 5000)   {
            stage = 2;
            gyro_heading_start = gyro.getHeading();
        }
         else if (stage == 2 &  gyro_heading_change > 85 ){
             stage = 1;
             encoder_start_l = left.getCurrentPosition();
        }

//      auto stuff
        if (stage == -1) {
         left.setPower(0.0);
         right.setPower(0.0);
     } else if (stage == 1) {
         left.setPower(.5);
         right.setPower(.5);
     } else if (stage == 2) {
            left.setPower(.5);
            right.setPower(-0.5);



     }


        //left.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x);
        //right.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x);


        backServo.setPosition(0.5 - 0.5* gamepad1.left_stick_y);
        telemetry.addData("Left_y","%f", gamepad1.left_stick_y);
        telemetry.addData("right_x", "%f", gamepad1.right_stick_x);
        telemetry.addData("Press", "Y-fwd, A-rev, B-Rt, X-Lt");
        telemetry.addData("Left Gamepad stick controls back servo","");
        telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
        telemetry.addData("Heading"," %.1f", gyro.getHeading());
        telemetry.addData("heading Change:", "%f",  gyro_heading_change);
        telemetry.addData("Encoders","Left %d  Right %d", left.getCurrentPosition(), right.getCurrentPosition());
        telemetry.addData("Distance", " Fr %.1f  Lt %.1f  Rt %.1f  Bk %.1f  ",
                frontDistance.getDistance(DistanceUnit.CM), leftDistance.getDistance(DistanceUnit.CM),
                rightDistance.getDistance(DistanceUnit.CM), backDistance.getDistance(DistanceUnit.CM));



    }

}
