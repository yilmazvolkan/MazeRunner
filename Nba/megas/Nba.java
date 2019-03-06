package megas;
import java.awt.Point;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;
import java.util.Stack;

import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.motor.BaseRegulatedMotor;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.NXTLightSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.Color;
import lejos.robotics.ColorAdapter;
import lejos.robotics.LightDetectorAdaptor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;
import lejos.utility.PilotProps;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;


public class Nba {
	
	// =================================================================
	// ========================== ANGLES ===============================
	// =================================================================
		
	public static final double TURN_RIGHT_ANGLE = 75.0;
	public static final double TURN_LEFT_ANGLE = -75.0;
	public static final int GRASP_ANGLE = 140;
	public static final int RELEASE_ANGLE = -180;
	
	
	// =================================================================
	// ========================== DISTANCES ============================
	// =================================================================
	public static final int HALF_BLOCK = 12;
	public static final int FULL_BLOCK = 31;
	public static final int QUARTER_BLOCK = 6;

	
	// =================================================================
	// ========================== THRESHOLDS ===========================
	// =================================================================
	public static final int ANGLE_CORRECTION_THRESHOLD = 1;
	
	// =================================================================
	// ======================== PILOT PROPS ============================
	// =================================================================
	public static final int LINEAR_SPEED = 5;
	public static final int ANGULAR_SPEED = 25;

	// =================================================================
	// ========================== ENUMS ================================
	// =================================================================		
	public static final int NXT_RED_MODE = 0;
	
	// =================================================================
	// ================== MIDDLE MOTOR SPEEDS ==========================
	// =================================================================		
	public static final int MIDDLE_MOTOR_SLOW_SPEED = 100;
	public static final int MIDDLE_MOTOR_SPEED = 200;
	
	// =================================================================
	// ============= ULTRASONIC NXT MOTOR ROTATIONS ====================
	// =================================================================		
	public static final int ULTRASONIC_ROTATE_RIGHT = 90;
	public static final int ULTRASONIC_ROTATE_LEFT = -90;
	public static final int MEASUREMENT_NUMBER = 5;
	public static final float WALL_DISTANCE = 30.0f;
	
	// =================================================================
	// ==================== GYRO ROTATING ANGLEs =======================
	// =================================================================	
	public static final float RIGHT_ROTATE_GYRO_ANGLE = 90.0f;
	public static final float LEFT_ROTATE_GYRO_ANGLE = -90.0f;

	static EV3 ev3 = (EV3) BrickFinder.getDefault();
	
	// =================================================================
	// ========================== SENSORS ==============================
	// =================================================================
	static EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S1);
	// static NXTLightSensor nxtLightSensor = new NXTLightSensor(SensorPort.S2);
	static EV3ColorSensor ev3ColorSensor = new EV3ColorSensor(SensorPort.S4);
	static EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S3);

	static ColorAdapter ev3ColorAdapter = new ColorAdapter(ev3ColorSensor);
	// static LightDetectorAdaptor nxtLightDetectorAdaptor = new LightDetectorAdaptor((SampleProvider)nxtLightSensor);
	
	
	// ====================================================================
	// ======================= POSITION INFO ==============================
	// ====================================================================
	static int orientation = 0;
	static int xPos = 3;
	static int yPos = 3;
	
	private static final String filepath="./map_file";

	
	public static final int MAPPING_MODE = 0;
	public static final int LOCALIZATION_MODE = 1;
	public static final int GO_TO_BALL_MODE = 2;

	static int current_mod = MAPPING_MODE; // mapping
	

	static MovePilot pilot;
	static GraphicsLCD graphicsLCD;

	static Map map;
	static boolean isFinished = false;
	static int ballColor = Utils.RED_BALL_CODE; 	// DEFAULT BALL COLOR IS RED
	
	static Utils utils = new Utils();
	
	public static void main(String[] args) throws Exception {		
		EV3 ev3 = (EV3) BrickFinder.getDefault();
		graphicsLCD = ev3.getGraphicsLCD();
		
		map = new Map();
		
		graphicsLCD.clear();
		graphicsLCD.drawString("Nba", graphicsLCD.getWidth()/2, 0, GraphicsLCD.VCENTER|GraphicsLCD.HCENTER);
		
		PilotProps pilotProps = new PilotProps();
		pilotProps.setProperty(PilotProps.KEY_WHEELDIAMETER, "4.96");
		pilotProps.setProperty(PilotProps.KEY_TRACKWIDTH, "11.94");
		pilotProps.setProperty(PilotProps.KEY_LEFTMOTOR, "B");
		pilotProps.setProperty(PilotProps.KEY_RIGHTMOTOR, "D");
		pilotProps.setProperty(PilotProps.KEY_REVERSE, "false");
		pilotProps.storePersistentValues();
		pilotProps.loadPersistentValues();
    	
    	EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.B);
    	EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.D);
    	EV3LargeRegulatedMotor ultrasonicSensorMotor = new EV3LargeRegulatedMotor(MotorPort.A);
    	EV3MediumRegulatedMotor middleMotor = new EV3MediumRegulatedMotor(MotorPort.C);
    	
    	float leftWheelDiameter = Float.parseFloat(pilotProps.getProperty(PilotProps.KEY_WHEELDIAMETER, "5.72"));
    	float rightWheelDiameter = Float.parseFloat(pilotProps.getProperty(PilotProps.KEY_WHEELDIAMETER, "5.28"));
    	
    	float trackWidth = Float.parseFloat(pilotProps.getProperty(PilotProps.KEY_TRACKWIDTH, "11.95"));
    	boolean reverse = Boolean.parseBoolean(pilotProps.getProperty(PilotProps.KEY_REVERSE, "false"));
    	
    	Chassis chassis = new WheeledChassis(new Wheel[]{WheeledChassis.modelWheel(leftMotor,leftWheelDiameter).offset(-trackWidth/2).invert(reverse),WheeledChassis.modelWheel(rightMotor,rightWheelDiameter).offset(trackWidth/2).invert(reverse)}, WheeledChassis.TYPE_DIFFERENTIAL);
    	
    	pilot = new MovePilot(chassis);
    	pilot.setLinearSpeed(LINEAR_SPEED);
    	pilot.setAngularSpeed(ANGULAR_SPEED);
    	pilot.stop();
		
		ServerSocket serverSocket = new ServerSocket(1234);
		
		graphicsLCD.clear();
		graphicsLCD.drawString("Nba", graphicsLCD.getWidth()/2, 0, GraphicsLCD.VCENTER|GraphicsLCD.HCENTER);
		graphicsLCD.drawString("Waiting", graphicsLCD.getWidth()/2, 20, GraphicsLCD.VCENTER|GraphicsLCD.HCENTER);
		graphicsLCD.refresh();
		
		Socket client = serverSocket.accept();
		
		graphicsLCD.clear();
		graphicsLCD.drawString("Nba", graphicsLCD.getWidth()/2, 0, GraphicsLCD.VCENTER|GraphicsLCD.HCENTER);
		graphicsLCD.drawString("Connected", graphicsLCD.getWidth()/2, 20, GraphicsLCD.VCENTER|GraphicsLCD.HCENTER);
		graphicsLCD.refresh();
        
		InputStream inputStream = client.getInputStream();
		DataInputStream dataInputStream = new DataInputStream(inputStream);
		
		OutputStream outputStream = client.getOutputStream();
		DataOutputStream dataOutputStream = new DataOutputStream(outputStream);
		
		while(!isFinished) {
			int buttonId = Button.readButtons();
			if ( buttonId == Button.ID_UP && !isFinished) {
				System.out.println("Mapping and Localization is starting...");
				executeMapping(dataOutputStream,ultrasonicSensorMotor);
			} else if (buttonId == Button.ID_DOWN && !isFinished) {
				System.out.println("Task execution is starting...");
				executeTaskExecution(dataOutputStream, middleMotor ,ultrasonicSensorMotor);
			} 
		}
		
		/*
		// ============ MAP MAKING ============ 
		Map map = new Map();
		current_mod = MAPPING_MODE;
		dataOutputStream.writeInt(current_mod);
		dataOutputStream.flush();
		System.out.println(map.toString());
		map = dfs(ultrasonicSensorMotor, dataOutputStream);
		map.writeObjectToFile(filepath);
		System.out.println(map.toString());
		
		graphicsLCD.clear();
		graphicsLCD.drawString("PRESS TO LOCALIZE", graphicsLCD.getWidth()/2, 0, GraphicsLCD.VCENTER|GraphicsLCD.HCENTER);
		graphicsLCD.refresh();
		Button.waitForAnyPress();
		*/
		
		/*
		// ============ LOCALIZATION ============ 
		current_mod = LOCALIZATION_MODE;
		
		dataOutputStream.writeInt(current_mod);
		dataOutputStream.flush();
		map = map.ReadObjectFromFile(filepath);
		System.out.println(map.toString());
		sendMap(dataOutputStream, map);
		graphicsLCD.clear();
		graphicsLCD.drawString("MAP IS LOADED", graphicsLCD.getWidth()/2, 0, GraphicsLCD.VCENTER|GraphicsLCD.HCENTER);
		graphicsLCD.refresh();
		localize(ultrasonicSensorMotor, dataOutputStream, map);
		*/
		
		/*
		// ============= GO TO BALL =============
		current_mod = GO_TO_BALL_MODE;
		dataOutputStream.writeInt(current_mod);
		dataOutputStream.flush();
		
		goToBall(dataOutputStream, map);
		*/
		dataOutputStream.close();
		serverSocket.close();
		
		
	}
	
	public static void executeTaskExecution(DataOutputStream dataOutputStream, EV3MediumRegulatedMotor mediumMotor,EV3LargeRegulatedMotor ultrasonicSensorMotor) throws IOException {
		
		// LOCALIZATION
		current_mod = LOCALIZATION_MODE;
		
		dataOutputStream.writeInt(current_mod);
		dataOutputStream.flush();
		map = map.ReadObjectFromFile(filepath);
		System.out.println(map.toString());
		sendMap(dataOutputStream, map);
		
		graphicsLCD.clear();
		graphicsLCD.drawString("MAP IS LOADED", graphicsLCD.getWidth()/2, 0, GraphicsLCD.VCENTER|GraphicsLCD.HCENTER);
		graphicsLCD.refresh();
		
		localize(ultrasonicSensorMotor, dataOutputStream, map);
		
		// Go to Ball
		current_mod = GO_TO_BALL_MODE;
		dataOutputStream.writeInt(current_mod);
		dataOutputStream.flush();
		
		goToGetTheBall(dataOutputStream, mediumMotor, map);
		goToDropTheBall(dataOutputStream, mediumMotor, map, ballColor);

		goToGetTheBall(dataOutputStream, mediumMotor, map);
		goToDropTheBall(dataOutputStream, mediumMotor, map, ballColor);
		
		Point p = new Point(xPos,yPos);
		turnBackToGreen(dataOutputStream, p, mediumMotor);
		makeMappingFinishSound();
	}
	
	public static void turnBackToGreen(DataOutputStream dataOutputStream, Point coordinates, EV3MediumRegulatedMotor mediumMotor) throws IOException {
		int xStart = coordinates.x;
		int yStart = coordinates.y;
		
		sendPositionDataOnPath(dataOutputStream);
		Point start_point = new Point(xStart,yStart);
		goFromTo(dataOutputStream, mediumMotor, map, start_point, map.green_coordinates, false, false);
	}
	
	public static void executeMapping(DataOutputStream dataOutputStream, EV3LargeRegulatedMotor ultrasonicSensorMotor) throws IOException {
		
		// MAPPING
		current_mod = MAPPING_MODE;
		
		dataOutputStream.writeInt(current_mod);
		dataOutputStream.flush();
		System.out.println(map.toString());
		map = dfs(ultrasonicSensorMotor, dataOutputStream);
		map.writeObjectToFile(filepath);
		System.out.println(map.toString());
		
		makeMappingFinishSound();
	}
	
	public static void makeMappingFinishSound() {
		Sound.playTone(500, 500);
		Sound.playTone(500, 500);
		Sound.playTone(500, 500);
	}
	
	public static Cell explore(EV3LargeRegulatedMotor ultrasonicSensorMotor, DataOutputStream dataOutputStream) throws IOException{
		boolean[] walls = new boolean[4];
		int colorId = ev3ColorAdapter.getColorID();
		if (colorId!=7) {
			// Take the measurements
			// Front
			int wall_readings = 0;
			for(int i=0 ; i < MEASUREMENT_NUMBER ; i++) {
				float measurement = getUltrasonicSensorValue();
				if (measurement < WALL_DISTANCE) {
					wall_readings++;
				}
			}
			if (wall_readings > 2) {
				walls[(0 + orientation)%4] = true;
			}
			else {
				walls[(0 + orientation)%4] = false;
			}
			wall_readings = 0;
			ultrasonicSensorMotor.rotate(ULTRASONIC_ROTATE_RIGHT);
			// Right
			for(int i=0 ; i < MEASUREMENT_NUMBER ; i++) {
				float measurement = getUltrasonicSensorValue();
				if (measurement < WALL_DISTANCE) {
					wall_readings++;
				}
			}
			if (wall_readings > 2) {
				walls[(1 + orientation)%4] = true;
			}
			else {
				walls[(1 + orientation)%4] = false;
			}
			wall_readings = 0;
			ultrasonicSensorMotor.rotate(ULTRASONIC_ROTATE_RIGHT);
			// Back
			for(int i=0 ; i < MEASUREMENT_NUMBER ; i++) {
				float measurement = getUltrasonicSensorValue();
				if (measurement < WALL_DISTANCE) {
					wall_readings++;
				}
			}
			if (wall_readings > 2) {
				walls[(2 + orientation)%4] = true;
			}
			else {
				walls[(2 + orientation)%4] = false;
			}
			wall_readings = 0;
			ultrasonicSensorMotor.rotate(3 * ULTRASONIC_ROTATE_LEFT);
			// Left
			for(int i=0 ; i < MEASUREMENT_NUMBER ; i++) {
				float measurement = getUltrasonicSensorValue();
				if (measurement < WALL_DISTANCE) {
					wall_readings++;
				}
			}
			if (wall_readings > 2) {
				walls[(3 + orientation)%4] = true;
			}
			else {
				walls[(3 + orientation)%4] = false;
			}
			wall_readings = 0;
			ultrasonicSensorMotor.rotate(ULTRASONIC_ROTATE_RIGHT);			
		}

		Cell cell = new Cell(colorId, walls);
		
		// Send the cell data to draw the map
		sendPositionData(dataOutputStream, cell);
		return cell;
	}
	
	public static void grabTheBall(GraphicsLCD graphicsLCD, EV3MediumRegulatedMotor middleMotor) {	
		middleMotor.setSpeed(MIDDLE_MOTOR_SPEED);
		middleMotor.rotate(RELEASE_ANGLE);
		middleMotor.stop();
		middleMotor.setSpeed(MIDDLE_MOTOR_SLOW_SPEED);
		pilot.travel(FULL_BLOCK-QUARTER_BLOCK);	// TODO: This might need to be increased.
		middleMotor.rotate(GRASP_ANGLE);
		middleMotor.stop();
		pilot.travel(-FULL_BLOCK+QUARTER_BLOCK);
		
		// If it determines the ball's color then updates the static field.
		ballColor = Utils.determineBallColor(graphicsLCD);
		System.out.println("BALL COLOR IS DETERMINED AS: " + ballColor);
	}
	
	public static void letTheBall(GraphicsLCD graphicsLCD, EV3MediumRegulatedMotor middleMotor) {
		
		pilot.travel(QUARTER_BLOCK);	// TODO: This might need to be DECREASED.
		
		/*middleMotor.setSpeed(MIDDLE_MOTOR_SLOW_SPEED);
		middleMotor.rotate(GRASP_ANGLE);
		middleMotor.stop();*/
		
		//Release the claw again.
		middleMotor.setSpeed(MIDDLE_MOTOR_SPEED);
		middleMotor.rotate(RELEASE_ANGLE);
		middleMotor.stop();
		pilot.travel(-QUARTER_BLOCK);
	}
	
	public static void sendPositionData(DataOutputStream dataOutputStream, Cell currentCell) throws IOException {
		
		dataOutputStream.writeInt(current_mod);
		dataOutputStream.flush();
		dataOutputStream.writeInt(xPos);
		dataOutputStream.flush();
		dataOutputStream.writeInt(yPos);
		dataOutputStream.flush();		
		dataOutputStream.writeInt(orientation);
		dataOutputStream.flush();
		dataOutputStream.writeInt(currentCell.colorId);
		dataOutputStream.flush();
		dataOutputStream.writeBoolean(currentCell.frontWall);
		dataOutputStream.flush();
		dataOutputStream.writeBoolean(currentCell.rightWall);
		dataOutputStream.flush();
		dataOutputStream.writeBoolean(currentCell.backWall);
		dataOutputStream.flush();
		dataOutputStream.writeBoolean(currentCell.leftWall);
		dataOutputStream.flush();

	}
	
	public static float getUltrasonicSensorValue() {
		SampleProvider sampleProvider = ultrasonicSensor.getDistanceMode();
		if(sampleProvider.sampleSize() > 0) {
			float [] samples = new float[sampleProvider.sampleSize()];
			sampleProvider.fetchSample(samples, 0);
			return samples[0]*100;
		}
		return -1;
	}
	
	public static void turnRight() {
		orientation = (orientation +1) % 4;
		gyroSensor.reset();
		pilot.rotate(TURN_RIGHT_ANGLE);
		float gyro_value = getGyroSensorValue();
		if(gyro_value > RIGHT_ROTATE_GYRO_ANGLE + 1) {
			pilot.rotate(RIGHT_ROTATE_GYRO_ANGLE - gyro_value);
		}else if (gyro_value < RIGHT_ROTATE_GYRO_ANGLE - 1) {
			pilot.rotate(RIGHT_ROTATE_GYRO_ANGLE - gyro_value);
		}
		
		gyroSensor.reset();

	}
	
	public static void turnLeft() {
		if(orientation == 0) {
			orientation = 3;
		}else {
			orientation = (orientation -1) % 4;	
		}
		gyroSensor.reset();
		pilot.rotate(TURN_LEFT_ANGLE);
		float gyro_value = getGyroSensorValue();
		
		if(gyro_value > LEFT_ROTATE_GYRO_ANGLE + 1) {
			pilot.rotate(LEFT_ROTATE_GYRO_ANGLE - gyro_value);
		}else if (gyro_value < LEFT_ROTATE_GYRO_ANGLE - 1) {
			pilot.rotate(LEFT_ROTATE_GYRO_ANGLE - gyro_value);
		}
		
		gyroSensor.reset();

	}
	
	public static void goForward(int distance) {
		gyroSensor.reset();
		pilot.travel(distance);
		float gyro_value = getGyroSensorValue();
		if (Math.abs(gyro_value)>ANGLE_CORRECTION_THRESHOLD) {
			pilot.rotate(-gyro_value);
		}
		gyroSensor.reset();
		
		
	}
	
	public static float getGyroSensorValue() {
	    SampleProvider sampleProvider = gyroSensor.getAngleAndRateMode();
	    //while(sampleProvider.sampleSize() == 0);
		float [] sample = new float[sampleProvider.sampleSize()];
    	sampleProvider.fetchSample(sample, 0);
    	float angle = sample[0];
    	return -1*angle;
	}
	
	public static Map dfs(EV3LargeRegulatedMotor ultrasonicSensorMotor, DataOutputStream dataOutputStream) throws IOException {
		Map map = new Map();
		Point orijin = new Point(xPos, yPos);
		Stack<Point> stack = new Stack<Point>();
		stack.push(orijin);
		ArrayList<Point> traversed = new ArrayList<Point>();
		traversed.add(orijin);
		Point current_coordinates = orijin;
		
		Stack<Integer> traversed_directions = new Stack<Integer>();

		
		while(!stack.isEmpty()) {
			Point new_coordinates = stack.pop();
			int manhattan_distance = Math.abs(new_coordinates.x - current_coordinates.x) + Math.abs(new_coordinates.y - current_coordinates.y);
		
			System.out.println("===========================");
			System.out.println("Current x: " + current_coordinates.x + "Current y: " + current_coordinates.y);
			System.out.println("new x: " + new_coordinates.x + "new y: " + new_coordinates.y);
			System.out.println("Manhattan Distance: " + manhattan_distance);
			System.out.println("===========================");
			while(manhattan_distance > 1 || checkIfThereIsWallBetween(map, current_coordinates,new_coordinates)) {	// Burasý kesin büyüktür 1 olmalý.
				
				int direction = traversed_directions.pop();
				System.out.println("#########################");
				System.out.println("Manhattan Distance: " + manhattan_distance);
				System.out.println("Direction: " + direction);
				System.out.println("Pre Orientation: " + orientation);
				System.out.println("#########################");
				
				if (direction == 0) {
					changeOrientationAndGoUp();
					System.out.println("-----------------");
					System.out.println("GO UP");
					current_coordinates.x = current_coordinates.x - 1;
				} else if (direction == 1) {
					changeOrientationAndGoRight();
					System.out.println("-----------------");
					System.out.println("GO RIGHT");
					current_coordinates.y = current_coordinates.y + 1;
				} else if (direction == 2) {
					System.out.println("-----------------");
					System.out.println("GO DOWN");
					changeOrientationAndGoDown();
					current_coordinates.x = current_coordinates.x + 1;
				} else if (direction == 3) {
					System.out.println("-----------------");
					System.out.println("GO LEFT");
					changeOrientationAndGoLeft();
					current_coordinates.y = current_coordinates.y - 1;
				}
				System.out.println("-----------------");
				System.out.println("After turning Orientation: " + orientation);
				manhattan_distance = Math.abs(new_coordinates.x - current_coordinates.x) + Math.abs(new_coordinates.y - current_coordinates.y);
			}
			
			if(new_coordinates.x < current_coordinates.x){
				// Go up
				System.out.println("GOING FORWARD");
				traversed_directions.push(2);
				changeOrientationAndGoUp();
			}
			else if(new_coordinates.x > current_coordinates.x) {
				// Go down
				System.out.println("GOING DOWN");
				traversed_directions.push(0);
				changeOrientationAndGoDown();
			}
			else if(new_coordinates.y > current_coordinates.y) {
				// Go right
				System.out.println("GOING RIGHT");
				traversed_directions.push(3);
				changeOrientationAndGoRight();
			}
			else if(new_coordinates.y < current_coordinates.y) {
				// Go left
				System.out.println("GOING LEFT");
				traversed_directions.push(1);
				changeOrientationAndGoLeft();
			}
		
			Point prev_coordinates = current_coordinates;
			current_coordinates = new_coordinates;
			xPos = current_coordinates.x;
			yPos = current_coordinates.y;
			traversed.add(current_coordinates);
			
			Cell current_cell = explore(ultrasonicSensorMotor, dataOutputStream);
			if(current_cell.colorId == 1) { //GREEN
				map.green_coordinates = new Point(current_coordinates.x, current_coordinates.y);
			} 
			else if (current_cell.colorId == 0){ //RED
				map.red_coordinates = new Point(current_coordinates.x, current_coordinates.y);
			}
			else if (current_cell.colorId == 2){ //BLUE
				map.blue_coordinates = new Point(current_coordinates.x, current_coordinates.y);
			}
			map.addCell(current_cell, current_coordinates.x, current_coordinates.y);
			if (current_cell.colorId != 7) {
				if(!current_cell.frontWall) {
					Point forward_coordinates = new Point(current_coordinates.x -1 , current_coordinates.y);
					if(!traversed.contains(forward_coordinates)) {
						stack.push(forward_coordinates);
					}
				}
				if(!current_cell.rightWall) {
					Point right_coordinates = new Point(current_coordinates.x , current_coordinates.y+1);
					if(!traversed.contains(right_coordinates)) {
						stack.push(right_coordinates);
					}			
				}
				if(!current_cell.backWall) {
					Point back_coordinates = new Point(current_coordinates.x +1 , current_coordinates.y);
					if(!traversed.contains(back_coordinates)) {
						stack.push(back_coordinates);
					}
				}
				if(!current_cell.leftWall) {
					Point left_coordinates = new Point(current_coordinates.x , current_coordinates.y-1);
					if(!traversed.contains(left_coordinates)) {
						stack.push(left_coordinates);
					}
				}
			}else {
				goForward(-FULL_BLOCK);
				traversed_directions.pop();
				current_coordinates = prev_coordinates;
			}
		}
		return map;
	}
	
	public static boolean checkIfThereIsWallBetween(Map map, Point current, Point next) {
		Cell currentCell = map.getCellAt(current.x, current.y);
		Cell nextCell = map.getCellAt(next.x, next.y);
		
		boolean ret = false;
		if (current.x > next.x) {
			ret = currentCell.frontWall;
		} else if (current.x < next.x) {
			ret = currentCell.backWall;
		} else if (current.y > next.y) {
			ret = currentCell.leftWall;
		} else if (current.y < next.y) {
			ret = currentCell.rightWall;
		}
		System.out.println("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");
		System.out.println("Current x: " + current.x + "Current y: " + current.y);
		System.out.println("next x: " + next.x + "next y: " + next.y);
		System.out.println(ret);
		System.out.println("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");
		return ret;
	}
	
	public static void changeOrientationAndGoUp() {
		if(orientation==1) {
			turnLeft();
		}
		else if(orientation==2) {
			turnRight();
			turnRight();
		}
		else if(orientation==3) {
			turnRight();
		}
		
		goForward(FULL_BLOCK);
		orientation = 0;
	}
	
	public static void changeOrientationAndGoDown() {
		if(orientation==3) {
			turnLeft();
		}
		else if(orientation==0) {
			turnRight();
			turnRight();
		}
		else if(orientation==1) {
			turnRight();
		}
		
		goForward(FULL_BLOCK);
		orientation = 2;

	}
	
	public static void changeOrientationAndGoRight() {
		if(orientation==2) {
			turnLeft();
		}
		else if(orientation==3) {
			turnRight();
			turnRight();
		}
		else if(orientation==0) {
			turnRight();
		}

		goForward(FULL_BLOCK);
		orientation = 1;

	}
	
	public static void changeOrientationAndGoLeft() {
		if(orientation==0) {
			turnLeft();
		}
		else if(orientation==1) {
			turnRight();
			turnRight();
		}
		else if(orientation==2) {
			turnRight();
		}
		
		goForward(FULL_BLOCK);
		orientation = 3;
	}
	
	// This method is used for only the last movement of the route to grab or leave the ball
	public static void changeOrientationToUpAndTurn() {
		if(orientation==1) {
			turnLeft();
		}
		else if(orientation==2) {
			turnRight();
			turnRight();
		}
		else if(orientation==3) {
			turnRight();
		}
		
		orientation = 0;
	}
	
	// This method is used for only the last movement of the route to grab or leave the ball
	public static void changeOrientationToDownAndTurn() {
		if(orientation==3) {
			turnLeft();
		}
		else if(orientation==0) {
			turnRight();
			turnRight();
		}
		else if(orientation==1) {
			turnRight();
		}
		
		orientation = 2;

	}
	
	// This method is used for only the last movement of the route to grab or leave the ball
	public static void changeOrientationToRightAndTurn() {
		if(orientation==2) {
			turnLeft();
		}
		else if(orientation==3) {
			turnRight();
			turnRight();
		}
		else if(orientation==0) {
			turnRight();
		}

		orientation = 1;
	}
	
	// This method is used for only the last movement of the route to grab or leave the ball
	public static void changeOrientationToLeftAndTurn() {
		if(orientation==0) {
			turnLeft();
		}
		else if(orientation==1) {
			turnRight();
			turnRight();
		}
		else if(orientation==2) {
			turnRight();
		}
		
		orientation = 3;
	}
	
	public static void localize(EV3LargeRegulatedMotor ultrasonicSensorMotor, DataOutputStream dataOutputStream, Map map) throws IOException {
		orientation = 0;
		System.out.println(" LOCALIZATION STARTS ");
		ArrayList<int[]> particles = new ArrayList<int[]>();
		for(int i = 0; i< map.MAP_WIDTH; i++) {
			for(int j = 0; j<map.MAP_WIDTH; j++) {
				if(map.getCellAt(i, j).colorId != -1) {	// Yani mapda ise
					for(int k = 0; k<4; k++ ) {
						int[] particle = {i, j, k};	// k= orientation Her yöne bakan 4 farklý partikül oluþtur gönder.
						particles.add(particle);
					}
				}
			}
		}
		
		System.out.println("PARTICLES SIZE " + particles.size());
		// Eliminate Particles until we find where we are.
		int previous_direction = -1;
		while(particles.size() > 1) {	// TODO
			sendParticles(particles, dataOutputStream);
			Cell current_cell = exploreParticles(ultrasonicSensorMotor, particles, map);
			sendParticles(particles, dataOutputStream);
			if(particles.size() > 1){
				// Robotu ve particlelarý update et hareket edip
				System.out.println("Previous: " + previous_direction);

				if(!current_cell.frontWall && (previous_direction!=0)) {
					System.out.println("Current: " + 2 );
					changeOrientationAndGoUp();
					orientation=0;
					int turnDirection = 0;
					moveParticles(particles, turnDirection);
					previous_direction = turnDirection;
				}
				else if(!current_cell.rightWall && (previous_direction!=1)){
					System.out.println("Current: " + 3 );
					changeOrientationAndGoRight();
					orientation=0;
					int turnDirection = 1;
					moveParticles(particles, turnDirection);
					previous_direction = turnDirection;
				}
				else if(!current_cell.backWall && (previous_direction!=2)){
					System.out.println("Current: " + 0 );
					changeOrientationAndGoDown();
					orientation=0;
					int turnDirection = 2;
					moveParticles(particles, turnDirection);
					previous_direction = turnDirection;
				}
				else if(!current_cell.leftWall && (previous_direction!=3)){
					System.out.println("Current: " + 1 );
					changeOrientationAndGoLeft();
					orientation=0;
					int turnDirection = 3;
					moveParticles(particles, turnDirection);
					previous_direction = turnDirection;
				}
			}
		}
		System.out.println("LAST PARTICLE");
		sendParticles(particles, dataOutputStream);
		System.out.println(particles.size());
		Delay.msDelay(500);
		int[] position = particles.get(0);
		xPos = position[0];
		yPos = position[1];
		orientation = position[2];
	}
	
	private static void sendMap(DataOutputStream dataOutputStream, Map map) throws IOException {
		dataOutputStream.writeInt(current_mod);
		dataOutputStream.flush();
		for(int i = 0; i< map.MAP_WIDTH; i++) {
			for(int j = 0; j<map.MAP_WIDTH; j++) {
				Cell current_cell = map.getCellAt(i, j);
				dataOutputStream.writeInt(i); //x
				dataOutputStream.flush();
				dataOutputStream.writeInt(j); //y
				dataOutputStream.flush();
				dataOutputStream.writeInt(current_cell.colorId); // COLOR
				dataOutputStream.flush();
				dataOutputStream.writeBoolean(current_cell.frontWall); // Front Wall
				dataOutputStream.flush();
				dataOutputStream.writeBoolean(current_cell.rightWall); // Right Wall
				dataOutputStream.flush();
				dataOutputStream.writeBoolean(current_cell.backWall); // Back Wall
				dataOutputStream.flush();
				dataOutputStream.writeBoolean(current_cell.leftWall); // Left Wall
				dataOutputStream.flush();

				if((i == map.MAP_WIDTH -1) &&(j ==map.MAP_WIDTH)) {
					dataOutputStream.writeBoolean(false); // Not Ended
				}else {
					dataOutputStream.writeBoolean(true); // Ended
				}
				dataOutputStream.flush();

				
			}
		}
		
		
	}

	private static void sendParticles(ArrayList<int[]> particles, DataOutputStream dataOutputStream) throws IOException {
		System.out.println("-----sendParticles-----");
		dataOutputStream.writeInt(current_mod);
		dataOutputStream.flush();
		ListIterator<int[]> iterator = particles.listIterator();
		while(iterator.hasNext()) {
			int[] current_particle = iterator.next();
			dataOutputStream.writeInt(current_particle[0]); //x
			dataOutputStream.flush();
			dataOutputStream.writeInt(current_particle[1]); //y
			dataOutputStream.flush();
			dataOutputStream.writeInt(current_particle[2]); //orientation
			dataOutputStream.flush();
			if(iterator.hasNext()) {
				dataOutputStream.writeBoolean(false); //not ended
			}
			else {
				dataOutputStream.writeBoolean(true); //ended
			}
			dataOutputStream.flush();
		}
		System.out.println("-----sendParticles Over -----");

	}

	public static void moveParticles(ArrayList<int[]> particles, int turnDirection) {
		if (turnDirection==1) {
			for(int i = 0; i < particles.size(); i++) {
				int [] current_particle = particles.get(i);
				current_particle[2] = (current_particle[2] + 1) %4;
				particles.set(i, current_particle);
			}
		}else if(turnDirection==3) {
			for(int i = 0; i < particles.size(); i++) {
				int [] current_particle = particles.get(i);
				if(current_particle[2] == 0) {
					current_particle[2] = 3;
				}
				else {
					current_particle[2] = (current_particle[2] -1) %4;
				}
				particles.set(i, current_particle);
			}
		}else if(turnDirection==2) {
			for(int i = 0; i < particles.size(); i++) {
				int [] current_particle = particles.get(i);
				current_particle[2] = (current_particle[2] +2) %4;
				particles.set(i, current_particle);
			}
		}	
		for(int i = 0; i < particles.size(); i++) {
			int [] current_particle = particles.get(i);
			if(current_particle[2] == 0) {
				current_particle[0] -= 1;
				particles.set(i, current_particle);
			}else if(current_particle[2] == 1) {
				current_particle[1] += 1;
				particles.set(i, current_particle);
			}else if(current_particle[2] == 2) {
				current_particle[0] += 1;
				particles.set(i, current_particle);
			}else if(current_particle[2] == 3) {
				current_particle[1] -= 1;
				particles.set(i, current_particle);
				}
			}
	/*
		if(turnDirection == 0) {
			for(int i = 0; i < particles.size(); i++) {
				int [] current_particle = particles.get(i);
				if(current_particle[2] == 0) {
					current_particle[0] -= 1;
					particles.set(i, current_particle);
				}else if(current_particle[2] == 1) {
					current_particle[1] += 1;
					particles.set(i, current_particle);
				}else if(current_particle[2] == 2) {
					current_particle[0] += 1;
					particles.set(i, current_particle);
				}else if(current_particle[2] == 3) {
					current_particle[1] -= 1;
					particles.set(i, current_particle);
					}
				}
			}
		
		else if(turnDirection == 1) {
			for(int i = 0; i < particles.size(); i++) {
				int [] current_particle = particles.get(i);
				if(current_particle[2] == 0) {
					current_particle[1] += 1;
					particles.set(i, current_particle);
				}else if(current_particle[2] == 1) {
					current_particle[0] += 1;
					particles.set(i, current_particle);
				}else if(current_particle[2] == 2) {
					current_particle[1] -= 1;
					particles.set(i, current_particle);
				}else if(current_particle[2] == 3) {
					current_particle[0] -= 1;
					particles.set(i, current_particle);
					}
				}
			}
		
		else if(turnDirection == 2) {
			for(int i = 0; i < particles.size(); i++) {
				int [] current_particle = particles.get(i);
				if(current_particle[2] == 0) {
					current_particle[0] += 1;
					particles.set(i, current_particle);
				}else if(current_particle[2] == 1) {
					current_particle[1] -= 1;
					particles.set(i, current_particle);
				}else if(current_particle[2] == 2) {
					current_particle[0] -= 1;
					particles.set(i, current_particle);
				}else if(current_particle[2] == 3) {
					current_particle[1] += 1;
					particles.set(i, current_particle);
					}
				}
			}
		else if(turnDirection == 3) {
			for(int i = 0; i < particles.size(); i++) {
				int [] current_particle = particles.get(i);
				if(current_particle[2] == 0) {
					current_particle[1] -= 1;
					particles.set(i, current_particle);
				}else if(current_particle[2] == 1) {
					current_particle[0] -= 1;
					particles.set(i, current_particle);
				}else if(current_particle[2] == 2) {
					current_particle[1] += 1;
					particles.set(i, current_particle);
				}else if(current_particle[2] == 3) {
					current_particle[0] += 1;
					particles.set(i, current_particle);
					}
				}
			*/
			
	}
	
	public static Cell exploreParticles(EV3LargeRegulatedMotor ultrasonicSensorMotor,
			ArrayList<int[]> particles,
			Map map
			) throws IOException{
		boolean[] walls = new boolean[4];
		int colorId = ev3ColorAdapter.getColorID();
		
		// Get the wall readings.
		if (colorId != 7) {
			// Take the measurements
			// Front
			int wall_readings = 0;
			for(int i=0 ; i < MEASUREMENT_NUMBER ; i++) {
				float measurement = getUltrasonicSensorValue();
				if (measurement < WALL_DISTANCE) {
					wall_readings++;
				}
			}
			if (wall_readings > 2) {
				walls[(0 + orientation) % 4] = true;
			}
			else {
				walls[(0 + orientation) % 4] = false;
			}
			wall_readings = 0;
			ultrasonicSensorMotor.rotate(ULTRASONIC_ROTATE_RIGHT);
			// Right
			for(int i=0 ; i < MEASUREMENT_NUMBER ; i++) {
				float measurement = getUltrasonicSensorValue();
				if (measurement < WALL_DISTANCE) {
					wall_readings++;
				}
			}
			if (wall_readings > 2) {
				walls[(1 ) % 4] = true;
			}
			else {
				walls[(1 ) % 4] = false;
			}
			wall_readings = 0;
			ultrasonicSensorMotor.rotate(ULTRASONIC_ROTATE_RIGHT);
			// Back
			for(int i=0 ; i < MEASUREMENT_NUMBER ; i++) {
				float measurement = getUltrasonicSensorValue();
				if (measurement < WALL_DISTANCE) {
					wall_readings++;
				}
			}
			if (wall_readings > 2) {
				walls[(2 ) % 4] = true;
			}
			else {
				walls[(2 ) % 4] = false;
			}
			wall_readings = 0;
			ultrasonicSensorMotor.rotate(3 * ULTRASONIC_ROTATE_LEFT);
			// Left
			for(int i=0 ; i < MEASUREMENT_NUMBER ; i++) {
				float measurement = getUltrasonicSensorValue();
				if (measurement < WALL_DISTANCE) {
					wall_readings++;
				}
			}
			if (wall_readings > 2) {
				walls[(3 ) % 4] = true;
			}
			else {
				walls[(3 ) % 4] = false;
			}
			wall_readings = 0;
			ultrasonicSensorMotor.rotate(ULTRASONIC_ROTATE_RIGHT);			
		}
		System.out.println("--------------PARTICLE ELIMINATION-----------");
		ListIterator<int[]> iterator = particles.listIterator();	// TODO Caným sýkýldý.
		while(iterator.hasNext()) {
			int [] current_particle = iterator.next();
			int particle_orientation = current_particle[2];
			
			Cell particle_cell = map.getCellAt(current_particle[0], current_particle[1]);
			System.out.println(" CURRENT PARTICLE ");
			System.out.println(current_particle[0] + " " + current_particle[1] + " " + current_particle[2]);
			
			// Eliminate the particle if the color is wrong.
			if (particle_cell.colorId !=colorId) {
				System.out.println(" REMOVED COLOR ");
				iterator.remove();
			}
			else {
				
				if(current_particle[2] == 0) {
					if(walls[0] != particle_cell.frontWall) {
						iterator.remove();
					}
					else if(walls[1] != particle_cell.rightWall){
						iterator.remove();
					}
					else if(walls[2] != particle_cell.backWall){
						iterator.remove();
					}
					else if(walls[3] != particle_cell.leftWall){
						iterator.remove();
					}
				}
				else if(current_particle[2] == 1) {
					if(walls[0] != particle_cell.rightWall) {
						iterator.remove();
					}
					else if(walls[1] != particle_cell.backWall){
						iterator.remove();
					}
					else if(walls[2] != particle_cell.leftWall){
						iterator.remove();
					}
					else if(walls[3] != particle_cell.frontWall){
						iterator.remove();
					}
				}
				else if(current_particle[2] == 2) {
					if(walls[0] != particle_cell.backWall) {
						iterator.remove();
					}
					else if(walls[1] != particle_cell.leftWall){
						iterator.remove();
					}
					else if(walls[2] != particle_cell.frontWall){
						iterator.remove();
					}
					else if(walls[3] != particle_cell.rightWall){
						iterator.remove();
					}
				}
				else if(current_particle[2] == 3) {
					if(walls[0] != particle_cell.leftWall) {
						iterator.remove();
					}
					else if(walls[1] != particle_cell.frontWall){
						iterator.remove();
					}
					else if(walls[2] != particle_cell.rightWall){
						iterator.remove();
					}
					else if(walls[3] != particle_cell.backWall){
						iterator.remove();
					}
				}
				/*
				// TODO: Orientationlar ayný olmayabilir.
				// TODO: 
				
				int[] wallCheck = new int[4];
				if(particle_orientation == 0) {
					wallCheck[0] = 0;
					wallCheck[1] = 1;
					wallCheck[2] = 2;
					wallCheck[3] = 3;
				} else if(particle_orientation == 1) {
					wallCheck[0] = 3;
					wallCheck[1] = 0;
					wallCheck[2] = 1;
					wallCheck[3] = 2;
				} else if(particle_orientation == 2) {
					wallCheck[0] = 2;
					wallCheck[1] = 3;
					wallCheck[2] = 0;
					wallCheck[3] = 1;
				} else if(particle_orientation == 3) {
					wallCheck[0] = 1;
					wallCheck[1] = 2;
					wallCheck[2] = 3;
					wallCheck[3] = 0;
				}
				
				int probabilities = 4;
				for(int i=0; i<4;i++) {
					System.out.println(" PROBABLILITY " + i);
					if(walls[(wallCheck[0]+i)%4] != particle_cell.frontWall) {
						System.out.println(" REMOVED FRONT");
						probabilities --;
					}
					else if(walls[(wallCheck[1]+i)%4] != particle_cell.rightWall) {
						System.out.println(" REMOVED RIGHT");
						probabilities --;
					}
					else if(walls[(wallCheck[2]+i)%4] != particle_cell.backWall) {
						System.out.println(" REMOVED BACK");
						probabilities --;
					}
					else if(walls[(wallCheck[3]+i)%4] != particle_cell.leftWall) {
						System.out.println(" REMOVED LEFT");
						probabilities --;
					}
				}
				if(probabilities == 0) {
					iterator.remove();
				}
				
				 	*/
			}
		}
		Cell cell = new Cell(colorId, walls);
		return cell;
	}	
	
	public static void goToGetTheBall(DataOutputStream dataOutputStream,EV3MediumRegulatedMotor mediumMotor,Map map) throws IOException {
		System.out.println("GO TO GRAB BALL ENTERED");
		
		Point end_point = map.green_coordinates;
		System.out.println("xPos: " + xPos);
		System.out.println("yPos: " + yPos);
		Point start_point = new Point(xPos, yPos);
		boolean grab_ball = true;
		boolean let_go_ball = false;
		
		goFromTo(dataOutputStream, mediumMotor ,map, start_point, end_point, grab_ball, let_go_ball);
		System.out.println("GO TO GRAB BALL EXIT");
	}
	
	public static void goToDropTheBall(DataOutputStream dataOutputStream, EV3MediumRegulatedMotor mediumMotor, Map map, int ballColor) throws IOException {
		System.out.println("GO TO DROP BALL ENTERED");
		
		Point end_point;
		
		// Determine where to go
		if (ballColor == Utils.BLUE_BALL_CODE) {
			end_point = map.blue_coordinates;	
		} else {
			end_point = map.red_coordinates;	
		}

		System.out.println("xPos: " + xPos);
		System.out.println("yPos: " + yPos);
		Point start_point = new Point(xPos, yPos);
		boolean grab_ball = false;
		boolean let_go_ball = true;
		
		goFromTo(dataOutputStream,mediumMotor, map, start_point, end_point, grab_ball, let_go_ball);
		System.out.println("GO TO DROP BALL EXIT");
	}
	
	public static void goFromTo(DataOutputStream dataOutputStream,
			EV3MediumRegulatedMotor mediumMotor,
			Map map,
			Point start_point,
			Point end_point,
			boolean grab_ball,
			boolean let_go_ball) throws IOException {
		System.out.println("GO FROM TO ENTER");

		ArrayList<Integer> route = pathFinder(map, start_point, end_point);
		for(int i = 0; i < route.size() - 1; i++) { 	// TODO: Could be wrong because I don't go to the last point but I need to turn towards it.
			
			sendPositionDataOnPath(dataOutputStream);
			
			if(route.get(i) == 0){
				xPos = xPos - 1;
				changeOrientationAndGoUp();
			}
			else if(route.get(i) == 1) {
				yPos = yPos + 1;
				changeOrientationAndGoRight();
			}
			else if(route.get(i) == 2) {
				xPos = xPos + 1;
				changeOrientationAndGoDown();
			}
			else if(route.get(i) == 3) {
				yPos = yPos - 1;
				changeOrientationAndGoLeft();
			}
			sendPositionDataOnPath(dataOutputStream);
		}
		
		// Solve the problem here: Either grab the ball or leave it.
		if (grab_ball) {
			int lastMovement = route.get(route.size() - 1);
			if(lastMovement == 0){
				// xPos = xPos - 1;
				changeOrientationToUpAndTurn();
			}
			else if(lastMovement == 1) {
				// yPos = yPos + 1;
				changeOrientationToRightAndTurn();
			}
			else if(lastMovement == 2) {
				// xPos = xPos + 1;
				changeOrientationToDownAndTurn();
			}
			else if(lastMovement == 3) {
				// yPos = yPos - 1;
				changeOrientationToLeftAndTurn();
			}
			
			sendPositionDataOnPath(dataOutputStream); // This might be problematic.
			// Then grab the ball
			grabTheBall(graphicsLCD,mediumMotor);
			
		} else if(let_go_ball) {
			int lastMovement = route.get(route.size() - 1);
			if(lastMovement == 0){
				// xPos = xPos - 1;
				changeOrientationToUpAndTurn();
			}
			else if(lastMovement == 1) {
				// yPos = yPos + 1;
				changeOrientationToRightAndTurn();
			}
			else if(lastMovement == 2) {
				// xPos = xPos + 1;
				changeOrientationToDownAndTurn();
			}
			else if(lastMovement == 3) {
				// yPos = yPos - 1;
				changeOrientationToLeftAndTurn();
			}
			
			sendPositionDataOnPath(dataOutputStream); // This might be problematic.
			// Then let the ball
			letTheBall(graphicsLCD,mediumMotor);
		} 
		// Only entered when turning back to green after basketing.
		else {
			sendPositionDataOnPath(dataOutputStream);
			int lastMovement = route.get(route.size() - 1);
			
			if(lastMovement == 0){
				xPos = xPos - 1;
				changeOrientationAndGoUp();
			}
			else if(lastMovement == 1) {
				yPos = yPos + 1;
				changeOrientationAndGoRight();
			}
			else if(lastMovement == 2) {
				xPos = xPos + 1;
				changeOrientationAndGoDown();
			}
			else if(lastMovement == 3) {
				yPos = yPos - 1;
				changeOrientationAndGoLeft();
			}
			sendPositionDataOnPath(dataOutputStream);
		}
		
		
		System.out.println("GO FROM TO EXIT");

	}
	
	public static ArrayList<Integer> pathFinder(Map map, Point start_point, Point end_point){
		Point current_point = start_point;
		Stack<Point> cell_stack = new Stack<Point>();
		ArrayList<int[]> parent_list = new ArrayList<int[]>();
		
		cell_stack.push(current_point);
		Stack<Point> traversed_stack = new Stack<Point>();

		System.out.println("END POINT : " + end_point.x + " " + end_point.y);
		while(!cell_stack.isEmpty()) {
			current_point = cell_stack.pop();
			traversed_stack.push(current_point);
			System.out.println("CURRENT POINT : " + current_point.x + " " + current_point.y);
			Cell current_cell = map.getCellAt(current_point.x, current_point.y);
			if(current_cell.colorId != 7) {
				if(!current_cell.frontWall) {
					Point child = new Point(current_point.x-1, current_point.y);
					int[] parent_info = {current_point.x, current_point.y, child.x, child.y, 0};
					parent_list.add(parent_info);
					if(!cell_stack.contains(child) && !traversed_stack.contains(child)) {
						cell_stack.push(child);
					}
					System.out.println("CHILD POINT : " + child.x + " " + child.y);
					if(child.equals(end_point)) {
						break;
					}
					//TO DO CHECK IF END BREAK
				}
				if(!current_cell.rightWall) {
					Point child = new Point(current_point.x, current_point.y+1);
					int[] parent_info = {current_point.x, current_point.y, child.x, child.y, 1};
					parent_list.add(parent_info);
					if(!cell_stack.contains(child)&& !traversed_stack.contains(child)) {
						cell_stack.push(child);
					}
					System.out.println("CHILD POINT : " + child.x + " " + child.y);
					if(child.equals(end_point)) {
						break;
					}
				}
				if(!current_cell.backWall) {
					Point child = new Point(current_point.x+1, current_point.y);
					int[] parent_info = {current_point.x, current_point.y, child.x, child.y, 2};
					parent_list.add(parent_info);
					if(!cell_stack.contains(child)&& !traversed_stack.contains(child)) {
						cell_stack.push(child);
					}
					System.out.println("CHILD POINT : " + child.x + " " + child.y);
					if(child.equals(end_point)) {
						break;
					}
				}
				if(!current_cell.leftWall) {
					Point child = new Point(current_point.x, current_point.y-1);
					int[] parent_info = {current_point.x, current_point.y, child.x, child.y, 3};
					parent_list.add(parent_info);
					if(!cell_stack.contains(child) && !traversed_stack.contains(child)) {
						cell_stack.push(child);
					}
					System.out.println("CHILD POINT : " + child.x + " " + child.y);
					if(child.equals(end_point)) {
						break;
					}
				}
			}
		}

		ArrayList<Integer> route = pathConstruct(parent_list, start_point, end_point);
		return route;
	}

	private static ArrayList<Integer> pathConstruct(ArrayList<int[]> parent_list, Point start_point, Point end_point) {
		Point current_point = end_point;
		ArrayList<Integer> path = new ArrayList<Integer>();
		
		while(!current_point.equals(start_point)) {
			System.out.println(",,,,,,,,,,,,,,,,,,,");
			System.out.println("START: " + start_point.x + " " + start_point.y );
			System.out.println("CURRENT: " + current_point.x + " " + current_point.y );
			System.out.println(",,,,,,,,,,,,,,,,,,,");

			for(int i = 0; i < parent_list.size(); i++) {
				int[] currentListItem = parent_list.get(i);
				// Construct the points from the parent list item (array).
				Point cur_first = new Point(currentListItem[0],currentListItem[1]);
				Point cur_second = new Point(currentListItem[2],currentListItem[3]);
				System.out.println("CUR_SECOND: " + cur_second.x + " " + cur_second.y );
				if (cur_second.equals(current_point)) {
					path.add(0, currentListItem[4]);	// Add the direction
					current_point = cur_first;
					break;
				}
			}
		}
		
		return path;
	}
	
	public static void sendPositionDataOnPath(DataOutputStream dataOutputStream) throws IOException {
		System.out.println("Send Position Data On Path");
		dataOutputStream.writeInt(current_mod);
		dataOutputStream.flush();
		dataOutputStream.writeInt(xPos);
		dataOutputStream.flush();
		dataOutputStream.writeInt(yPos);
		dataOutputStream.flush();
	}
}
