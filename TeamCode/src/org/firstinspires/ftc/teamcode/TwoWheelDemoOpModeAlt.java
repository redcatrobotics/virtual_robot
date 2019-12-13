package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "two wheel demo opmode Test", group = "TwoWheel")
public class TwoWheelDemoOpModeAlt extends OpMode {

    private DcMotor left = null;
    private DcMotor right = null;
    private GyroSensor gyro = null;
    private Servo backServo = null;
    private ColorSensor colorSensor = null;
    private DistanceSensor frontDistance = null;
    private DistanceSensor leftDistance = null;
    private DistanceSensor backDistance = null;
    private DistanceSensor rightDistance = null;

    private ElapsedTime et = null;
    private int waitForStartTime = 0;
    double last_red_ratio = 0.0;
    boolean halt = false;


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
    }

    public void init_loop(){
        if (et.milliseconds() >= 1000) {
            waitForStartTime++;
            et.reset();
        }
        telemetry.addData("Press Start to Continue"," %d", waitForStartTime);
    }

    public void loop(){

        double curr_et_sec = et.seconds();

        // the next three lines are not being used now - (John)
        double scaling_factor = 30.0 / (2.0 * 3.14159);
        double default_left_power  = 0.2 * Math.sin(curr_et_sec / scaling_factor);
        double default_right_power = 0.2 * Math.cos(curr_et_sec / scaling_factor);


        double red_ratio = colorSensor.red() / (0.5 * (colorSensor.green() + colorSensor.blue()));

        if (last_red_ratio > 1.4 & red_ratio < last_red_ratio) halt = true;

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
//            //left.setPower(default_left_power);
//            //right.setPower(default_right_power);
//            left.setPower(0.0);
//            right.setPower(0.0);
//            //left.setPower(.25);
//            //right.setPower(0.5);
//        }


        if (halt) {
            left.setPower(0);
            right.setPower(0);
        } else {
            left.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x);
            right.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x);
        }

        backServo.setPosition(0.5 - 0.5* gamepad1.left_stick_y);
        telemetry.addData("Press", "Y-fwd, A-rev, B-Rt, X-Lt");
        telemetry.addData("Left Gamepad stick controls back servo","");
        telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
        telemetry.addData("Red Ratio","Red Ratio %f", red_ratio);
        telemetry.addData("Heading"," %.1f", gyro.getHeading());
        telemetry.addData("Encoders","Left %d  Right %d", left.getCurrentPosition(), right.getCurrentPosition());
        telemetry.addData("Distance (cm)", " Fr %.1f  Lt %.1f  Rt %.1f  Bk %.1f  ",
                frontDistance.getDistance(DistanceUnit.CM), leftDistance.getDistance(DistanceUnit.CM),
                rightDistance.getDistance(DistanceUnit.CM), backDistance.getDistance(DistanceUnit.CM)
        );
        telemetry.addData("Elapsed Time (sec)", " %f", curr_et_sec);
        telemetry.addData("Default Power", "L: %f  R: %f", default_left_power, default_right_power);
        telemetry.addData("Left Gamepad Stick y,", "%f ",gamepad1.left_stick_y);
        telemetry.addData("Left gamepad stick x,","%f ",gamepad1.left_stick_x);

        last_red_ratio = red_ratio;  // set for next iteration
    }

}
