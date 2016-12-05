import cicontest.algorithm.abstracts.AbstractDriver;
import cicontest.algorithm.abstracts.DriversUtils;
import cicontest.torcs.controller.extras.ABS;
import cicontest.torcs.controller.extras.AutomatedClutch;
import cicontest.torcs.controller.extras.AutomatedGearbox;
import cicontest.torcs.controller.extras.AutomatedRecovering;
import cicontest.torcs.genome.IGenome;
import scr.Action;
import scr.SensorModel;

import java.io.InputStream;
import java.io.OutputStream;
import java.net.URL;
import java.net.URLConnection;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Random;
import java.util.WeakHashMap;





public class DefaultDriver extends AbstractDriver {

    private NeuralNetwork neuralNetwork;
    private LinkedList<Experience> replayMemory;
    private Random random;

    double[] previousState;
    double[] previousAction;
    double previousScore;

    private static final double SAFE_DISTANCE = 120;

    private static final double BRAKE_DISTANCE_MAX = 150;
    private static final double BRAKE_DISTANCE_MEDIUM = 60;
    private static final double BRAKE_DISTANCE_MIN = 30;

    private static final double HIGH_SPEED = 100;
    private static final double MEDIUM_SPEED = 40;
    private static final double SAFE_SPEED = 20;
    private static final double MINIMUM_SPEED = 30;

    private static final double ACCEL_FULL = 1.0;
    private static final double ACCEL_LOW = 0.5;


    private class Experience {
        double[] state, action, state2;
        double reward;
        boolean terminal;

        public Experience(double[] state, double[] action, double reward, double[] state2, boolean terminal) {
            this.state = state; this.action = action; this.reward = reward; this.state2 = state2; this.terminal = terminal;
        }
    }

    private void storeExperience(Experience experience) {
        replayMemory.add(experience);
        replayMemory.removeFirst();
    }

    private double[] getState(SensorModel sensors) {
        double[] currentState = new double[29];
        for (int i = 0; i < 19; i++) { currentState[i] = sensors.getTrackEdgeSensors()[i]; }
        for (int i = 0; i < 4; i++) { currentState[19 + i] = sensors.getWheelSpinVelocity()[i]; }
        currentState[23] = sensors.getAngleToTrackAxis();
        currentState[24] = sensors.getTrackPosition();
        currentState[25] = sensors.getZSpeed();
        currentState[26] = sensors.getSpeed();
        currentState[27] = sensors.getLateralSpeed();
        currentState[28] = sensors.getRPM();
        return currentState;
    }

    public DefaultDriver() {
        initialize();
        random = new Random();

        neuralNetwork = new NeuralNetwork(12, 8, 2);
//        neuralNetwork = neuralNetwork.loadGenome();

        // Replay memory
        replayMemory = new LinkedList<Experience>();
    }

    private void initialize() {
        this.enableExtras(new AutomatedClutch());
        this.enableExtras(new AutomatedGearbox());
        this.enableExtras(new AutomatedRecovering());
        this.enableExtras(new ABS());
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
        double[] output = neuralNetwork.getOutput(sensors);
        return 1;
    }

    @Override
    public double getSteering(SensorModel sensors) {
        double[] output = neuralNetwork.getOutput(sensors);
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
        final double MAX_SPEED = 60;
        final double CORNER_SPEED = 30;
        final double MAX_DIST = 100;

        double LOCK = Math.PI * .25;
        double OFFSET = 80;

        // Get sensor data
        double speed = sensors.getSpeed();
        double[] edgeSensors = sensors.getTrackEdgeSensors();
        double angle = sensors.getAngleToTrackAxis();
        double pos = sensors.getTrackPosition();

        double leftSensor = edgeSensors[4];
        double centerSensor = Math.max(Math.max(edgeSensors[8], edgeSensors[10]), edgeSensors[9]);
        double rightSensor = edgeSensors[14];

        // Calculate target speed
        double targetSpeed;
        if(centerSensor > MAX_DIST) {
            targetSpeed = MAX_SPEED;
        } else {
            targetSpeed = CORNER_SPEED;
        }

        action.accelerate = 2 / ((1 + Math.exp(speed - targetSpeed)) - 1);


        // Calculate steering
        double targetAngle = (angle - pos * .5);

        if (speed > OFFSET) {
            action.steering = targetAngle / (LOCK * (speed - OFFSET));
        } else {
            action.steering = targetAngle / LOCK;
        }

        action.limitValues();


        return action;
    }
/*
    @Override
    public Action defaultControl(Action action, SensorModel sensors) {
        if (action == null) action = new Action();

        final double SPEED_GOAL = 60;

        // Get current state
        double[] currentState = getState(sensors);

        // Slow drive
        if(currentState[26] < 50) {
            action.accelerate = 0.1;
        } else {
            action.accelerate = 0.0;
        }
        action.steering = 0.0;
        action.brake = 0.0;

        // Get reward
        double currentReward = Math.cos(currentState[23]) - Math.sin(currentState[23]) - currentState[24] - sensors.getDamage();
        if(sensors.getDamage() > 0) {
            action.restartRace = true;
        }


        // Store experience
        // Experience experience = new Experience(previousState, previousAction, currentReward, currentState, false);
        // storeExperience(experience);
        int act_num = 0;
        if(previousState != null && previousAction != null) {
            String str_s = "\"[[";
            String str_a = "\"[" + previousAction[0] + "," + previousAction[1] + "," + previousAction[2] + "," + previousAction[3] + "]\"";
            String str_r = "" + currentReward;
            String str_n = "\"[[";
            str_s += previousState[0];
            str_n += currentState[0];
            for (int i = 1; i < previousState.length; i++) {
                str_s += ", " + previousState[i];
                str_n += ", " + currentState[i];
            }
            str_s += "]]\"";
            str_n += "]]\"";


            try {
                String charset = "UTF-8";
                URLConnection connection = new URL("http://localhost:5000/post_experience").openConnection();
                connection.setDoOutput(true); // Triggers POST.
                connection.setRequestProperty("Content-Type", "application/json");
                String param = "{" +
                        "\"s\": " + str_s + "," +
                        "\"a\": " + str_a + "," +
                        "\"r\": " + str_r + "," +
                        "\"n\": " + str_n + "}";

                connection.setRequestProperty("data", param);

                try (OutputStream output = connection.getOutputStream()) {
                    output.write(param.getBytes(charset));
                }

                InputStream response = connection.getInputStream();
                act_num = response.read() - '0';

            } catch(Exception e) {
                System.out.println(e);
            }
        }

        System.out.println(act_num);
        switch(act_num) {
            case 0:
                action.steering = 0.;
                break;
            case 1:
                action.steering = -1.;
                break;
            case 2:
                action.steering = 1.;
                break;
            case 3:
                action.accelerate = .1;
                break;
        }


        // Get Action
        double[] currentAction = new double[4];
        currentAction[0] = 0.0;
        currentAction[1] = 0.0;
        currentAction[2] = 0.0;
        currentAction[3] = 0.0;
        currentAction[act_num] = 1.0;

        previousAction = currentAction;
        previousState = currentState;

        // Print action
        System.out.println("--------------" + getDriverName() + "--------------");
        System.out.println("Reward: " + currentReward);
        System.out.println("Steering: " + action.steering);
        System.out.println("Acceleration: " + action.accelerate);
        System.out.println("Brake: " + action.brake);
        System.out.println("-----------------------------------------------");

        return action;
    }
    */
}