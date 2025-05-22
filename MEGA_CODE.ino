#include <ros.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle nh;

// Encoder feedback message (x=right, y=left, z=0)
geometry_msgs::Vector3 encoder_msg;
ros::Publisher encoder_pub("/encoder", &encoder_msg);

void setup() {
  Serial.begin(57600);      // Debugging
  Serial1.begin(57600);      // TX1=18, RX1=19 (to Uno A0/A1)

  // Initialize ROS node
  nh.initNode();
  nh.advertise(encoder_pub);
}

void loop() {
  static int speedLeftCmd = 100;    // desired left speed
  static int speedRightCmd = -100;   // desired right speed

  // Send speeds to Uno (format: "LxxxRxxx")
  Serial1.print("L");
  Serial1.print(speedLeftCmd);
  Serial1.print("R");
  Serial1.print(speedRightCmd);
  Serial1.println();

  // Receive feedback from Uno (format: "FBLxxxRxxx")
  if (Serial1.available()) {
    String feedback = Serial1.readStringUntil('\n');
    if (feedback.startsWith("FB")) {
      int lPos = feedback.indexOf('L');
      int rPos = feedback.indexOf('R');
      if (lPos >= 0 && rPos >= 0) {
        // Parse feedback values
        encoder_msg.x = feedback.substring(rPos + 1).toFloat();  // Right motor (Rxxx)
        encoder_msg.y = feedback.substring(lPos + 1, rPos).toFloat();  // Left motor (Lxxx)
        encoder_msg.z = 0;  

        // Publish to ROS
        encoder_pub.publish(&encoder_msg);
        
        // Debug output
        Serial.print("Published to /encoder: ");
        Serial.print(encoder_msg.x);
        Serial.print(", ");
        Serial.print(encoder_msg.y);
        Serial.print(", ");
        Serial.println(encoder_msg.z);
      }
    }
  }

  nh.spinOnce();  
  delay(100);     // can be adjusted as needed
}
