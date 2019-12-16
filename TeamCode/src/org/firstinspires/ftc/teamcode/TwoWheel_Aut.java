package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "two wheel time based autonomous", group = "TwoWheel")
public class TwoWheel_Aut extends OpMode {

    private DcMotor left = null;
    private DcMotor right = null;
    private DcMotor left_intake = null;
    private DcMotor right_intake = null;
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
    double gyro_heading = 0.0;
    double last_gyro_heading = 0.0;
    double gyro_heading_change_at_transition = 0.0;
    double headingLastNonZeroDelta = 0.0;
    double left_trigger = 0.0;

    long stage = 1;
    long num_cycles = 0;
    long gyroNumHeadingChanges = 0;
    private ElapsedTime et = null;
    private int waitForStartTime = 0;

    double elapsed_time = 0.0;
    double power = 0.2;
    double rotation_direction = -1.0;

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

        elapsed_time = et.milliseconds() / 1000.0;
        telemetry.addData("Et (sec)", "%f", elapsed_time);

        if (elapsed_time < 3.0) {
            // go forward
            right.setPower(power);
            left.setPower(power);
            telemetry.addData("Mode", "%s", "1 - Forward1");

        } else if (elapsed_time < 5.25) {
            // turn left
            right.setPower(rotation_direction * power);
            left.setPower(-rotation_direction * power);
            telemetry.addData("Mode", "%s", "2 - Turn");

        } else if (elapsed_time < 10.55) {

            right.setPower(power);
            left.setPower(power);
            telemetry.addData("Mode", "%s", "3 - Forward");

        } else {
            left.setPower(0.0);
            right.setPower(0.0);
            telemetry.addData("Mode", "%s", "4 - Finished/Stopped");
        }
    }
}
