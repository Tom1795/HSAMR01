package parkingRobot.hsamr0;

import lejos.geom.Line;

import lejos.geom.Point;

import lejos.robotics.navigation.Pose;



import parkingRobot.INavigation;

import parkingRobot.INavigation.ParkingSlot.ParkingSlotStatus;

import parkingRobot.IPerception;

import parkingRobot.IMonitor;



import parkingRobot.hsamr0.NavigationThread;





/**

 * A executable basic example implementation of the corresponding interface provided by the Institute of Automation with

 * limited functionality:



 * <p>

 * In this example only the both encoder sensors from the {@link IPerception} implementation are used for periodically calculating

 * the robots position and corresponding heading angle (together called 'pose'). Neither any use of the map information or other

 * perception sensor information is used nor any parking slot detection is performed, although the necessary members are already

 * prepared. Advanced navigation calculation and parking slot detection should be developed and invented by the students.  

 * 

 * @author IfA

 */

public class NavigationAT implements INavigation{

	

	/**

	 * line information measured by right light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line

	 */

	int lineSensorRight	=	0;

	/**

	 * line information measured by left light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line

	 */

	int lineSensorLeft	=	0;

	

	/**

	 * reference to {@link IPerception.EncoderSensor} class for left robot wheel which measures the wheels angle difference

	 * between actual an last request

	 */

	IPerception.EncoderSensor encoderLeft							=	null;

	/**

	 * reference to {@link IPerception.EncoderSensor} class for right robot wheel which measures the wheels angle difference

	 * between actual an last request

	 */

	IPerception.EncoderSensor encoderRight							=	null;

	

	/**

	 * reference to data class for measurement of the left wheels angle difference between actual an last request and the

	 * corresponding time difference

	 */

	IPerception.AngleDifferenceMeasurement angleMeasurementLeft 	= 	null;

	/**

	 * reference to data class for measurement of the right wheels angle difference between actual an last request and the

	 * corresponding time difference

	 */

	IPerception.AngleDifferenceMeasurement angleMeasurementRight	= 	null;

	

	/**

	 * reference to {@link IPerception.OdoSensor} class for mouse odometry sensor to measure the ground displacement

	 * between actual and last request

	 */

	IPerception.OdoSensor mouseodo = null;	

		

	/**

	 * reference to data class for measurement of the mouse odometry sensor to measure the ground displacement between

	 * actual an last request and the corresponding time difference

	 */

	IPerception.OdoDifferenceMeasurement mouseOdoMeasurement = null;

	

	/**

	 * distance from optical sensor pointing in driving direction to obstacle in mm

	 */

	double frontSensorDistance		=	0;

	/**

	 * distance from optical sensor pointing to the right side of robot to obstacle in mm (sensor mounted at the front)

	 */

	double frontSideSensorDistance	=	0;

	/**

	 * distance from optical sensor pointing in opposite of driving direction to obstacle in mm

	 */

	double backSensorDistance		=	0;

	/**

	 * distance from optical sensor pointing to the right side of robot to obstacle in mm (sensor mounted at the back)

	 */

	double backSideSensorDistance	=	0;





	/**

	 * robot specific constant: radius of left wheel

	 */

	static final double LEFT_WHEEL_RADIUS	= 	0.028; // only rough guess, to be measured exactly and maybe refined by experiments

	/**

	 * robot specific constant: radius of right wheel

	 */

	static final double RIGHT_WHEEL_RADIUS	= 	0.028; // only rough guess, to be measured exactly and maybe refined by experiments

	/**

	 * robot specific constant: distance between wheels

	 */

	static final double WHEEL_DISTANCE		= 	0.114; // only rough guess, to be measured exactly and maybe refined by experiments

	

	

	

	

	

	

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	

	/**

	 * robot specific constant: Offset zum hinteren seitlichen Sensor

	 */

	static final float OFFSET_FRONT_SIDE_SENSOR = 5.00f;

	

	/**

	 * robot specific constant: Offset zum vorderen seitlichen Sensor

	 */

	static final float OFFSET_BACK_SIDE_SENSOR = 5.00f;

	

	/**

	 * Länge des Roboters

	 */

	static final double ROBOT_LENGTH = 23.00;  // richtige länge einsetzen

	

	/** 

	 * Maximaler (realer) Abstand zum Mittelpunkt der Linie in der Kurve 

	 * */

	final static double MAX_WALL_DISTANCE = 21.2132; // entspricht genauer maximaler länge. Offset fehlt!

	

	/**

	 * Minimaler Abstand, um eine Parklücke zu erkennen

	 */

	final static double MIN_SLOT_DISTANCE = 30.00;

	

	/**

	 * Maximale Abweichung zur Unterscheidung von gleichen Parklücken

	 */

	final static double SAME_PARKING_SLOT_DIFFERENCE = 5.00;

	

	

	/**

	 * Initialisierung ParkingSlot

	 */

	ParkingSlot currentParkingSlot = new ParkingSlot(0, null, null, null, 0);

	

	/**

	 * Initialisierung ParkingSlot Array

	 */

	INavigation.ParkingSlot parkingSlots[] = new ParkingSlot[10];			// hier werden die einzelenen Parklücken gespeichert. Initialisiert mit 10.

	

	/**

	 * ParkingSlot Zähler -> definiert Position im Array parkingSlots		

	 */

	int slotCounter = 0;													// Variable zum Zählen der gefunden Parklücken

	

	

	

	



	

	/**

	 * map array of line references, whose corresponding lines form a closed chain and represent the map of the robot course

	 */

	Line[] map 								= null;

	/**

	 * reference to the corresponding main module Perception class

	 */

	IPerception perception 	        		= null;

	/**

	 * reference to the corresponding main module Monitor class

	 */

	IMonitor monitor = null;

	/**

	 * indicates if parking slot detection should be switched on (true) or off (false)

	 */

	boolean parkingSlotDetectionIsOn		= false;

	/**

	 * pose class containing bundled current X and Y location and corresponding heading angle phi

	 */

	Pose pose								= new Pose();

	

	

	/**

	 * weitere Pose

	 */

	Pose unimprovedPose 					= new Pose();

	

	/**

	 * thread started by the 'Navigation' class for background calculating

	 */

	NavigationThread navThread = new NavigationThread(this);



	

	/**

	 * provides the reference transfer so that the class knows its corresponding perception object (to obtain the measurement

	 * information from) and starts the navigation thread.

	 * 

	 * @param perception corresponding main module Perception class object

	 * @param monitor corresponding main module Monitor class object

	 */

	public NavigationAT(IPerception perception, IMonitor monitor){

		this.perception   = perception;

		this.monitor = monitor;

		this.encoderLeft  = perception.getNavigationLeftEncoder();

		this.encoderRight = perception.getNavigationRightEncoder();

		this.mouseodo = perception.getNavigationOdo();		

		

		navThread.setPriority(Thread.MAX_PRIORITY - 1);

		navThread.setDaemon(true); // background thread that is not need to terminate in order for the user program to terminate

		navThread.start();

	}

	

	

	// Inputs

	

	/* (non-Javadoc)

	 * @see parkingRobot.INavigation#setMap(lejos.geom.Line[])

	 */

	public void setMap(Line[] map){

		this.map = map;

	}

	/* (non-Javadoc)

	 * @see parkingRobot.INavigation#setDetectionState(boolean)

	 */

	public void setDetectionState(boolean isOn){

		this.parkingSlotDetectionIsOn = isOn;

	}

	

	

	// 	Class control

	

	/* (non-Javadoc)

	 * @see parkingRobot.INavigation#updateNavigation()

	 */

	public synchronized void updateNavigation(){	

		this.updateSensors();

		this.calculateLocation();
		//this.setDetectionState(true);
		if (this.parkingSlotDetectionIsOn)

				this.detectParkingSlot();

		

		// MONITOR (example)

//		monitor.writeNavigationComment("Navigation");

	}

	

	

	// Outputs

	

	/* (non-Javadoc)

	 * @see parkingRobot.INavigation#getPose()

	 */

	public synchronized Pose getPose(){

		return this.pose;

	}

	/* (non-Javadoc)

	 * @see parkingRobot.INavigation#getParkingSlots()

	 */

	public synchronized ParkingSlot[] getParkingSlots() {		// Funktion gibt Anzahl der tatsächlichen Parklücken zurück.

																// <-> Array parkingSlots hat unter umständen mehrere leere Objekte, daher neues Array mit kopierten Werten.

		ParkingSlot actualNumberOfSlots[] = new ParkingSlot[slotCounter];

		System.arraycopy(parkingSlots, 0, actualNumberOfSlots, 0, slotCounter);

		return actualNumberOfSlots;

	}

	

	

	// Private methods

	

	/**

	 * calls the perception methods for obtaining actual measurement data and writes the data into members

	 */

	private void updateSensors(){		

		this.lineSensorRight		= perception.getRightLineSensor();

		this.lineSensorLeft  		= perception.getLeftLineSensor();

		

		this.angleMeasurementLeft  	= this.encoderLeft.getEncoderMeasurement();

		this.angleMeasurementRight 	= this.encoderRight.getEncoderMeasurement();



		this.mouseOdoMeasurement	= this.mouseodo.getOdoMeasurement();



		this.frontSensorDistance	= perception.getFrontSensorDistance();

		this.frontSideSensorDistance = perception.getFrontSideSensorDistance();

		this.backSensorDistance		= perception.getBackSensorDistance();

		this.backSideSensorDistance	= perception.getBackSideSensorDistance();

	}		 	

	

	/**

	 * calculates the robot pose from the measurements

	 */

	private void calculateLocation(){

		double leftAngleSpeed 	= this.angleMeasurementLeft.getAngleSum()  / ((double)this.angleMeasurementLeft.getDeltaT()/1000);  //degree/seconds

		double rightAngleSpeed 	= this.angleMeasurementRight.getAngleSum() / ((double)this.angleMeasurementRight.getDeltaT()/1000); //degree/seconds



		double vLeft		= (leftAngleSpeed  * Math.PI * LEFT_WHEEL_RADIUS ) / 180 ; //velocity of left  wheel in m/s

		double vRight		= (rightAngleSpeed * Math.PI * RIGHT_WHEEL_RADIUS) / 180 ; //velocity of right wheel in m/s		

		double w 			= (vRight - vLeft) / WHEEL_DISTANCE; //angular velocity of robot in rad/s

		

		Double R 			= new Double(( WHEEL_DISTANCE / 2 ) * ( (vLeft + vRight) / (vRight - vLeft) ));								

		

		double ICCx 		= 0;

		double ICCy 		= 0;



		double xResult 		= 0;

		double yResult 		= 0;

		double angleResult 	= 0;

		

		double deltaT       = ((double)this.angleMeasurementLeft.getDeltaT())/1000;

		

		if (R.isNaN()) { //robot don't move

			xResult			= this.pose.getX();

			yResult			= this.pose.getY();

			angleResult 	= this.pose.getHeading();

		} else if (R.isInfinite()) { //robot moves straight forward/backward, vLeft==vRight

			xResult			= this.pose.getX() + vLeft * Math.cos(this.pose.getHeading()) * deltaT;

			yResult			= this.pose.getY() + vLeft * Math.sin(this.pose.getHeading()) * deltaT;

			angleResult 	= this.pose.getHeading();

		} else {			

			ICCx = this.pose.getX() - R.doubleValue() * Math.sin(this.pose.getHeading());

			ICCy = this.pose.getY() + R.doubleValue() * Math.cos(this.pose.getHeading());

		

			xResult 		= Math.cos(w * deltaT) * (this.pose.getX()-ICCx) - Math.sin(w * deltaT) * (this.pose.getY() - ICCy) + ICCx;

			yResult 		= Math.sin(w * deltaT) * (this.pose.getX()-ICCx) + Math.cos(w * deltaT) * (this.pose.getY() - ICCy) + ICCy;

			angleResult 	= this.pose.getHeading() + w * deltaT;

		}

		

		this.pose.setLocation((float)xResult, (float)yResult);

		this.pose.setHeading((float)angleResult);		 

	}

	

	// ---- WORK IN PROGRESS ----

	private void improveHeadingIR() {

		if(Math.abs(this.pose.getHeading()) - 10 < 15) {	// Untere gerade linie U

			

		}

	}

	

	

	private void improveLocationOpticalMouse() {

		// Initialisierung + Reset der lokalen Variablen

		

		float xResult = 0;

		float yResult = 0;

		float betrag = 0;

		

		betrag = (float) Math.sqrt(Math.pow(unimprovedPose.getX() - pose.getX(), 2) + Math.pow(unimprovedPose.getY() - pose.getY(), 2));

		

		xResult = betrag * (float) Math.cos(pose.getHeading());

		yResult = betrag * (float) Math.sin(pose.getHeading());

		

		unimprovedPose.setLocation(xResult, yResult);

	}



	/**

	 * detects parking slots and manages them by initializing new slots, re-characterizing old slots or merge old and detected slots. 

	 */

	private void detectParkingSlot(){

		

		// backBoundary der Parklücke

		if(frontSensorDistance > MAX_WALL_DISTANCE) {

			Point boundary;

			

			boundary = new Point(pose.getX() + OFFSET_FRONT_SIDE_SENSOR, pose.getY());

			currentParkingSlot.setBackBoundaryPosition(boundary);

			monitor.writeNavigationComment("backBoundary gefunden: " + currentParkingSlot.getBackBoundaryPosition()); //debugging

		}

		

		// frontBoundary der Parklücke

		if(backSideSensorDistance < MIN_SLOT_DISTANCE) {

			

			Point boundary;

			

			boundary = new Point(pose.getX() - OFFSET_BACK_SIDE_SENSOR, pose.getY());

			currentParkingSlot.setFrontBoundaryPosition(boundary);

			monitor.writeNavigationComment("frontBoundary gefunden: " + currentParkingSlot.getBackBoundaryPosition()); //debugging

		}

		

		// Status der Parklücke

		if(Math.abs(currentParkingSlot.getBackBoundaryPosition().getX() - currentParkingSlot.getFrontBoundaryPosition().getX()) > ROBOT_LENGTH) {

			currentParkingSlot.setStatus(ParkingSlotStatus.SUITABLE_FOR_PARKING);

			monitor.writeNavigationComment("Parklücke passt! " + "ID= " + currentParkingSlot.getID() + " bBP= " + currentParkingSlot.getBackBoundaryPosition() + " fBP= " + currentParkingSlot.getFrontBoundaryPosition() + " status= " + currentParkingSlot.getStatus()); //debugging

		}

		else {

			currentParkingSlot.setStatus(ParkingSlotStatus.NOT_SUITABLE_FOR_PARKING);

			monitor.writeNavigationComment("Parklücke passt nicht! " + "ID= " + currentParkingSlot.getID() + " bBP= " + currentParkingSlot.getBackBoundaryPosition() + " fBP= " + currentParkingSlot.getFrontBoundaryPosition() + " status= " + currentParkingSlot.getStatus()); //debugging

		}

		

		

		

		//TODO Measurement Quality

		

		

		

		// ++++++++++++++++++++++++++++

		

		

		

		if(currentParkingSlot.getStatus() == ParkingSlotStatus.SUITABLE_FOR_PARKING) {

			// nur passende Parklücken werden gespeichert

			

			parkingSlots[slotCounter] = currentParkingSlot;

			slotCounter += 1;

			monitor.writeNavigationComment("Parklücke gespeichert auf " + currentParkingSlot.getID() + "!");

			

			// Reset currentParkingSlot

			currentParkingSlot.setID(currentParkingSlot.getID() + 1);

			currentParkingSlot.setBackBoundaryPosition(null);

			currentParkingSlot.setFrontBoundaryPosition(null);

			currentParkingSlot.setStatus(null);

			currentParkingSlot.setMeasurementQuality(0);

		}

		

		

		

		//return; // has to be implemented by students

	}

	

	// grober Aufbau der Vergleichsfunktion. Vorher muss noch durch das gesamte Array parkingSlots iteriert werden.. (Funktion steht noch nicht)

	/**

	 * Funktion zur Unterscheidung zwischen bereits gefundenen und neuen Parklücken

	 * @param a Parklücke 1 (bereits in Datenbank)

	 * @param b Parklücke 2 (neu gemessen)

	 * @return true wenn bekannt; false wenn unbekannt

	 */

	public boolean sameSlot ( ParkingSlot a, ParkingSlot b) {

		

		if(Math.abs(a.getBackBoundaryPosition().getX() - b.getFrontBoundaryPosition().getX()) < SAME_PARKING_SLOT_DIFFERENCE)

			return true;

		

		else return false;

		

	}

	

	

	/*

	/**

	 * Vergleicht Distanz zur Seite und prüft, ob es sich um eine Parklücke handelt.

	 * @param distance

	 * @return boolean

	 *

	private boolean distCheck(double distance) {

		if (distance > MIN_SLOT_DISTANCE) {

			return true;

		}

		else return false;

	}*/

}