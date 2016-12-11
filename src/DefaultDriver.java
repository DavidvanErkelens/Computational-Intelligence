import cicontest.algorithm.abstracts.AbstractDriver;
import cicontest.torcs.controller.extras.ABS;
import cicontest.torcs.controller.extras.AutomatedClutch;
import cicontest.torcs.controller.extras.AutomatedGearbox;
import cicontest.torcs.genome.IGenome;
import scr.Action;
import scr.SensorModel;
public class DefaultDriver extends AbstractDriver {

    private NeuralNetwork neuralNetwork;


    public DefaultDriver() {
        initialize();

        neuralNetwork = new NeuralNetwork(12, 8, 2);
    }

    private void initialize() {
        this.enableExtras(new AutomatedClutch());
        this.enableExtras(new AutomatedGearbox());
//        this.enableExtras(new ABS());
    }


    @Override
    public void loadGenome(IGenome genome) {
        if (genome instanceof DefaultDriverGenome) {
            DefaultDriverGenome myGenome = (DefaultDriverGenome) genome;
        } else {
            System.err.println("Invalid Genome assigned");
        }
    }

    @Override
    public double getAcceleration(SensorModel sensors) {
        double[] sensorArray = new double[4];
        double output = neuralNetwork.getOutput(sensors);
        return 1;
    }

    @Override
    public double getSteering(SensorModel sensors) {
        double output = neuralNetwork.getOutput(sensors);
        return 0.0;
    }

    @Override
    public String getDriverName() {
        return "TD Controller v1";
    }

    @Override
    public Action controlWarmUp(SensorModel sensors) {
        Action action = new Action();
        return defaultControl(action, sensors);
    }

    @Override
    public Action controlQualification(SensorModel sensors) {
        Action action = new Action();
        return defaultControl(action, sensors);
    }

    @Override
    public Action controlRace(SensorModel sensors) {
        Action action = new Action();
        return defaultControl(action, sensors);
    }

    @Override
    public Action defaultControl(Action action, SensorModel sensors) {
        // Set heuristics
        final double MAX_SPEED = 120;
        final double CORNER_SPEED = 60;
        final double MAX_DIST = 75;
        final double LONG_DIST = 100;
        final double OFFTRACK = 0.5;
        final double BRAKE_DIST = 90;
        final double BRAKE_SPEED = 70;
        final double BRAKE_POWER = 0.5;
        final double IGNORE_BEAM_DIST = 90;
        final double MAX_BRAKE_STEER = 0.35;
        final double STEER_BACK_TO_TRACK = 0.4;

        double LOCK = Math.PI * .25;

        // Get sensor data
        double speed = sensors.getSpeed();
        double[] edgeSensors = sensors.getTrackEdgeSensors();
        double angle = sensors.getAngleToTrackAxis();
        double pos = sensors.getTrackPosition();

        double centerSensor = 0.0;
        for (int i = 7; i <= 11; i++) centerSensor = Math.max(centerSensor, edgeSensors[i]);

        int maxdist = 9;
        boolean offtrack = false;

        for (int i = 0; i <= 18; i++) {
            if (edgeSensors[i] > IGNORE_BEAM_DIST) continue;
            if (edgeSensors[i] > edgeSensors[maxdist]) maxdist = i;
            if (edgeSensors[i] <= OFFTRACK) {
                offtrack = true;
                break;
            }
        }

        if (offtrack && edgeSensors[0] < OFFTRACK / 2.0) action.steering = -1 * STEER_BACK_TO_TRACK;
        else if (offtrack && edgeSensors[18] < OFFTRACK / 2.0) action.steering = STEER_BACK_TO_TRACK;
        else action.steering = (maxdist - 9) / -9.0;


        // Calculate target speed
        double targetSpeed;
        if (centerSensor > MAX_DIST) {
            targetSpeed = MAX_SPEED;
        } else if (centerSensor < BRAKE_DIST && sensors.getSpeed() > BRAKE_SPEED) {
            action.brake = BRAKE_POWER;
            targetSpeed = 0;
            if (Math.abs(action.steering) > MAX_BRAKE_STEER) {
                if (action.steering < 0) action.steering = -1 * MAX_BRAKE_STEER;
                else action.steering = MAX_BRAKE_STEER;
            }
        } else {
            targetSpeed = CORNER_SPEED;
        }

        action.accelerate = 2 / ((1 + Math.exp(speed - targetSpeed)) - 1);
        if (Math.max(edgeSensors[9], Math.max(edgeSensors[8], edgeSensors[10])) > LONG_DIST && !offtrack)
            action.accelerate = 1.0;

        action.limitValues();

        return action;
    }
//    @Override
//    public Action defaultControl(Action action, SensorModel sensors) {
//        // Set heuristics
//        final double MAX_SPEED = 120;               // maximum speed on straights
//        final double CORNER_SPEED = 60;             // maximum speed in corners
//        final double MAX_DIST = 75;                 // distance we need free to go full speed
//        final double LONG_DIST = 100;               // distance we need free to plankgas
//        final double OFFTRACK = 0.5;                // amount the side sensors need before steering back to the center
//        final double BRAKE_DIST = 90;               // distance we measure before a thight corner when we start to brake
//        final double BRAKE_SPEED = 70;              // the speed to which we brake before a corner
//        final double BRAKE_POWER = 0.5;             // the brake force
//        final double IGNORE_BEAM_DIST = 90;         // the distance from which we ignore beams when deciding where to steer
//        final double MAX_BRAKE_STEER = 0.35;        // the maximum amount of steering when braking
//        final double STEER_BACK_TO_TRACK = 0.55;     // the force with which we steer back on track when we drift off
//
//        // Get sensor data
//        double speed = sensors.getSpeed();
//        double[] edgeSensors = sensors.getTrackEdgeSensors();
//
//        double LOCK = Math.PI * .25;
//        double angle = sensors.getAngleToTrackAxis();
//        double pos = sensors.getTrackPosition();
//
//        double centerSensor = 0.0;
//        for (int i = 7; i <= 11; i++) centerSensor = Math.max(centerSensor, edgeSensors[i]);
//
//        int maxdist = 9;
//        boolean offtrack = false;
//
//        for (int i = 0; i <= 18; i++)
//        {
//            if (edgeSensors[i] > IGNORE_BEAM_DIST) continue;
//            if (edgeSensors[i] > edgeSensors[maxdist]) maxdist = i;
//            if (edgeSensors[i] <= OFFTRACK) {
//                offtrack = true;
//                break;
//            }
//        }
//
//        if (offtrack && edgeSensors[0] < OFFTRACK / 2.0) action.steering  = -1 * STEER_BACK_TO_TRACK;
//        else if (offtrack && edgeSensors[18] < OFFTRACK / 2.0) action.steering = STEER_BACK_TO_TRACK;
//        else action.steering = (maxdist - 9) / -9.0;
//
//
//        // Calculate target speed
//        double targetSpeed;
//        if(centerSensor > MAX_DIST) {
//            targetSpeed = MAX_SPEED;
//        } else if (centerSensor < BRAKE_DIST * (speed / 100.0) && speed > BRAKE_SPEED) {
//            action.brake = BRAKE_POWER;
//            targetSpeed = 0;
//            if (Math.abs(action.steering) > MAX_BRAKE_STEER)
//            {
//                if (action.steering < 0) action.steering = -1 * MAX_BRAKE_STEER;
//                else action.steering = MAX_BRAKE_STEER;
//            }
//        } else {
//            targetSpeed = CORNER_SPEED;
//        }
//
//        action.accelerate = 2 / ((1 + Math.exp(speed - targetSpeed)) - 1);
//        if (Math.max(edgeSensors[9], Math.max(edgeSensors[8], edgeSensors[10])) > LONG_DIST && !offtrack) action.accelerate = 1.0;
//
//        if (Math.abs(pos) >= 1 && sensors.getGear() < 0)
//        {
//            action.steering = (angle - pos * .5) / LOCK;
//            if (Math.abs(angle) >= Math.PI / 2.0) action.steering *= -1;
//            action.accelerate = 0.2;
//            action.gear = 1;
//
//        }
//        action.limitValues();
//
//        return action;
//    }
}