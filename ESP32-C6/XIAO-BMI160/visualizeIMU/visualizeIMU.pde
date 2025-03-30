import processing.serial.*;

Serial myPort;        // Serial port object
String inString;      // String for storing incoming data
float ax, ay, az;     // Accelerometer values (in g)
float gx, gy, gz;     // Gyroscope values (in rad/s)

// Ball properties
float ballX, ballY;   // Ball position
float ballSize = 40;  // Ball diameter
color ballColor = color(50, 150, 255);

// Motion trail properties
int maxTrailPoints = 100;
ArrayList<PVector> trailPoints = new ArrayList<PVector>();
float gyroScale = 5.0; // Scale factor for gyroscope visualization

// Canvas settings
int canvasWidth = 800;
int canvasHeight = 600;

// Mapping parameters
float accelRange = 2.0;  // Expected range of accelerometer values in g
float smoothFactor = 0.1; // Lower = smoother movement

// Serial port name
String portName = "COM6"; // Specific port for your ESP32

// Mode
boolean showGyroTrail = true;

void setup() {
  size(800, 600);
  
  // Initialize ball position to center of canvas
  ballX = width / 2;
  ballY = height / 2;
  
  // List all available serial ports
  println("Available serial ports:");
  printArray(Serial.list());
  
  // Connect to COM6
  try {
    myPort = new Serial(this, portName, 115200);
    myPort.bufferUntil('\n');  // Buffer until newline character
    println("Connected to: " + portName);
  } catch (Exception e) {
    println("Error opening serial port: " + e.getMessage());
    println("Please check if your ESP32 is connected to " + portName + " and reset the sketch.");
  }
}

void draw() {
  background(240);
  
  // Draw grid lines for reference
  drawGrid();
  
  // Draw motion trail
  drawMotionTrail();
  
  // Draw the ball
  fill(ballColor);
  noStroke();
  ellipse(ballX, ballY, ballSize, ballSize);
  
  // Draw sensor data display
  drawSensorData();
  
  // Display controls
  fill(80);
  textSize(14);
  textAlign(LEFT);
  text("Press SPACE to toggle between Gyro trail and Accel trail", 20, height - 20);
}

void keyPressed() {
  if (key == ' ') {
    showGyroTrail = !showGyroTrail;
    trailPoints.clear(); // Clear trail when switching modes
  }
}

void serialEvent(Serial myPort) {
  try {
    // Read the incoming data
    inString = myPort.readStringUntil('\n');
    
    if (inString != null) {
      // Trim whitespace
      inString = inString.trim();
      
      // Split the CSV string
      String[] values = split(inString, ',');
      
      // Check if we received all 6 values
      if (values.length >= 6) {
        // Parse the values and handle possible format errors
        try {
          float newAx = float(values[0]);
          float newAy = float(values[1]);
          float newAz = float(values[2]);
          float newGx = float(values[3]);
          float newGy = float(values[4]);
          float newGz = float(values[5]);
          
          // Check for NaN values (which can occur if parsing fails)
          if (!Float.isNaN(newAx) && !Float.isNaN(newAy) && !Float.isNaN(newAz) && 
              !Float.isNaN(newGx) && !Float.isNaN(newGy) && !Float.isNaN(newGz)) {
            
            // Update sensor values
            ax = newAx;
            ay = newAy;
            az = newAz;
            gx = newGx;
            gy = newGy;
            gz = newGz;
            
            // Map accelerometer data to ball position with smoothing
            // Note: We invert ax for natural left-right movement
            float targetX = map(-ax, -accelRange, accelRange, 0, width);
            float targetY = map(ay, -accelRange, accelRange, 0, height);
            
            // Apply smoothing
            ballX = lerp(ballX, targetX, smoothFactor);
            ballY = lerp(ballY, targetY, smoothFactor);
            
            // Keep ball within canvas boundaries
            ballX = constrain(ballX, ballSize/2, width - ballSize/2);
            ballY = constrain(ballY, ballSize/2, height - ballSize/2);
            
            // Add point to trail
            if (showGyroTrail) {
              // For gyro trail, add relative movement from rotation
              float lastX = (trailPoints.size() > 0) ? trailPoints.get(trailPoints.size()-1).x : width/2;
              float lastY = (trailPoints.size() > 0) ? trailPoints.get(trailPoints.size()-1).y : height/2;
              
              // Use gyroscope data to plot motion (rotation-based)
              float motionX = lastX + (gx * gyroScale);
              float motionY = lastY + (gy * gyroScale);
              
              // Keep points within bounds
              motionX = constrain(motionX, 0, width);
              motionY = constrain(motionY, 0, height);
              
              trailPoints.add(new PVector(motionX, motionY));
            } else {
              // For accel trail, just record the ball's position
              trailPoints.add(new PVector(ballX, ballY));
            }
            
            // Limit the trail length
            if (trailPoints.size() > maxTrailPoints) {
              trailPoints.remove(0);
            }
          }
        } catch (Exception e) {
          println("Error parsing data: " + e.getMessage());
        }
      }
    }
  } catch (Exception e) {
    println("Error in serialEvent: " + e.getMessage());
  }
}

void drawMotionTrail() {
  if (trailPoints.size() < 2) return;
  
  strokeWeight(3);
  
  // Draw lines between points with gradient color
  for (int i = 1; i < trailPoints.size(); i++) {
    PVector prev = trailPoints.get(i-1);
    PVector current = trailPoints.get(i);
    
    // Calculate color based on position in trail (newer = brighter)
    float colorFactor = map(i, 0, trailPoints.size(), 0, 1);
    
    if (showGyroTrail) {
      // Use purple for gyro trail
      stroke(100 + 155*colorFactor, 0, 160 + 95*colorFactor, 150 + 105*colorFactor);
    } else {
      // Use orange for accel trail
      stroke(255*colorFactor, 100 + 100*colorFactor, 0, 150 + 105*colorFactor);
    }
    
    line(prev.x, prev.y, current.x, current.y);
  }
}

void drawGrid() {
  stroke(200);
  strokeWeight(1);
  
  // Vertical grid lines
  for (int x = 0; x <= width; x += width/10) {
    line(x, 0, x, height);
  }
  
  // Horizontal grid lines
  for (int y = 0; y <= height; y += height/10) {
    line(0, y, width, y);
  }
  
  // Draw center lines
  stroke(150);
  strokeWeight(2);
  line(width/2, 0, width/2, height);
  line(0, height/2, width, height/2);
}

void drawSensorData() {
  fill(50);
  textSize(16);
  textAlign(LEFT);
  
  int xPos = 20;
  int yPos = 30;
  int lineHeight = 24;
  
  text("IMU Sensor Data:", xPos, yPos);
  yPos += lineHeight;
  
  // Accelerometer data
  text("Accelerometer (g):", xPos, yPos);
  yPos += lineHeight;
  text(String.format("Ax: %.2f", ax), xPos, yPos);
  yPos += lineHeight;
  text(String.format("Ay: %.2f", ay), xPos, yPos);
  yPos += lineHeight;
  text(String.format("Az: %.2f", az), xPos, yPos);
  yPos += lineHeight;
  
  // Gyroscope data
  text("Gyroscope (rad/s):", xPos, yPos);
  yPos += lineHeight;
  text(String.format("Gx: %.2f", gx), xPos, yPos);
  yPos += lineHeight;
  text(String.format("Gy: %.2f", gy), xPos, yPos);
  yPos += lineHeight;
  text(String.format("Gz: %.2f", gz), xPos, yPos);
  
  // Mode display
  yPos += lineHeight;
  if (showGyroTrail) {
    fill(128, 0, 255);
    text("Mode: Gyroscope Movement (rotation)", xPos, yPos);
  } else {
    fill(255, 128, 0);
    text("Mode: Accelerometer Movement (tilt)", xPos, yPos);
  }
  
  // Draw connection status
  fill(50);
  textAlign(RIGHT);
  if (myPort != null) {
    fill(0, 150, 0);
    text("Connected to " + portName, width - 20, 30);
  } else {
    fill(200, 0, 0);
    text("Not Connected", width - 20, 30);
  }
}

// When the sketch closes
void exit() {
  if (myPort != null) {
    myPort.stop();
  }
  super.exit();
}
