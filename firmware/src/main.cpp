#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <MQTT.h>
#include <Servo.h>
#include <ArduinoJson.h>

#include "config.hpp"
#include "ServoMod.hpp"

const char *mqttClientIdentifier = "arduino";
const char *mqttClientUsername = "arduino";
const char *mqttClientPassword = "Ffeirluke234";

IPAddress mqttBrokerHostname(192, 168, 1, 60);
uint16_t mqttBrokerPort = 1883;
byte ethernetMACAddress[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

const String TOPIC_START = "/nodes/arduino/servo/";
const String SUBTOPIC_SERVO_BUFFER_EMPTY = "/buffer/empty";
const String SUBTOPIC_SERVO_BUFFER_DRAIN = "/buffer/drain";
const String SUBTOPIC_SERVO_BUFFER_PUSH = "/buffer/push";

EthernetClient mqttEthernetClient;
MQTTClient mqttClient;

ServoMod servos[] = {
    ServoMod(3),
    ServoMod(4),
    ServoMod(5),
    ServoMod(6),
    ServoMod(7),
};
const int numberOfServos = sizeof(servos) / sizeof(ServoMod);

void errorHandler()
{
  while (true)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(300);
  }
}

void blockUntilEthernetLinked()
{
  if (Ethernet.linkStatus() == LinkOFF)
  {
    Serial.print(F("I: Ethernet cable not connected, waiting"));

    do
    {
      Serial.print(F("."));
      delay(500);
    } while (Ethernet.linkStatus() == LinkOFF);

    Serial.println(F("Connected!"));
  }
}

void setupEthernet()
{
  Ethernet.init(10);

  Ethernet.begin(ethernetMACAddress);

  if (Ethernet.hardwareStatus() == EthernetNoHardware)
  {
    Serial.println(F("E: Ethernet hardware not connected"));
    errorHandler();
  }

  blockUntilEthernetLinked();

  Serial.print(F("I: IPAddress: "));
  Serial.println(Ethernet.localIP());

  Serial.print(F("I: Gateway: "));
  Serial.println(Ethernet.gatewayIP());

  Serial.print(F("I: SubnetMask: "));
  Serial.println(Ethernet.subnetMask());
}

String generateBufferDrainTopic(int n)
{
  String topic;

  topic += TOPIC_START;
  topic += n;
  topic += SUBTOPIC_SERVO_BUFFER_DRAIN;

  return topic;
}

String generateBufferEmptyTopic(int n)
{
  String topic;

  topic += TOPIC_START;
  topic += n;
  topic += SUBTOPIC_SERVO_BUFFER_EMPTY;

  return topic;
}

String generateBufferDrainMessage(int need)
{
  String message;

  JsonDocument document;
  document["need"] = need;
  serializeJson(document, message);

  return message;
}

String generateBufferEmptyMessage()
{
  return "{}";
}

void servoBufferDrainCallback(int n, int need)
{
  const String topic = generateBufferDrainTopic(n);
  const String message = generateBufferDrainMessage(need);

  mqttClient.publish(topic, message, false, 2);
}

void servoBufferEmptyCallback(int n)
{
  const String topic = generateBufferEmptyTopic(n);
  const String message = generateBufferEmptyMessage();

  mqttClient.publish(topic, message, false, 2);
}

/// @brief Call the servo buffer drain callback for servo N.
/// @tparam N The servo number.
template<int N>
inline void servoBufferDrainCallbackT(int need)
{
  servoBufferDrainCallback(N, need);
}

/// @brief Call the servo buffer empty callback for servo N.
/// @tparam N The servo number.
template<int N>
inline void servoBufferEmptyCallbackT()
{
  servoBufferEmptyCallback(N);
}

template<int N>
inline void assignServoCallbacks() {
  // Get the servo and it's buffer.
  ServoMod &servo = servos[N];
  SpeedServoBuffer &buffer = servo.getBuffer();

  // Assign the callbacks.
  buffer.onDrain(servoBufferDrainCallbackT<N>);
  buffer.onEmpty(servoBufferEmptyCallbackT<N>);
}

void setupServos()
{
  // Assign the callbacks to the servos, we're doing this using templates
  //  because arduino lacks any support for anonymous functions.
  assignServoCallbacks<0>();
  assignServoCallbacks<1>();
  assignServoCallbacks<2>();
  assignServoCallbacks<3>();
  assignServoCallbacks<4>();

  // Set all the servo's up.
  for (ServoMod &servo : servos)
  {
    servo.setup();
  }
}

void onServoBufferPushMessage(int n, String &payload)
{
  // Deserialize the json document.
  JsonDocument document;
  if (deserializeJson(document, payload) != DeserializationError::Ok)
  {
    Serial.println(F("Failed to parse json payload"));
    errorHandler();
  }

  // Get the targets from the document.
  JsonArray targets = document["targets"];

  // Loop over the targets.
  for (JsonObject target : targets)
  {
    // Get the speed and the angle.
    const int angle = target["angle"];
    const int speed = target["speed"];

    // Create the target.
    const SpeedServoTarget speedServoTarget = {
        .angle = angle,
        .speed = speed,
    };

    // Attempt to append the servo target.
    if (!servos[n].getBuffer().push(speedServoTarget))
    {
      Serial.print(F("E: Angle buffer overflow for servo "));
      Serial.println(n);
      errorHandler();
    }
  }
}

void onMqttMessage(String &topic, String &payload)
{
  // Ensure the topic has a valid start.
  if (!topic.startsWith(TOPIC_START))
  {
    Serial.println(F("E: Received invalid topic"));
    errorHandler();
  }

  // Get the remainder of the topic (without the start).
  String remainder = topic.substring(TOPIC_START.length());

  // Get the index of the next slash (will determine which servo to pick).
  const int slashIdx = remainder.indexOf('/');
  if (slashIdx == -1)
  {
    Serial.println(F("E: Received invalid topic, failed to get servo index."));
    errorHandler();
  }

  // Get the servo index.
  const String servoIndexString = remainder.substring(0, slashIdx);
  remainder = remainder.substring(slashIdx);

  // Parse the servo index.
  const int servoIndex = servoIndexString.toInt();

  // Make sure the servo index is valid.
  if (servoIndex < 0 || servoIndex >= numberOfServos)
  {
    Serial.println(F("E: Invalid servo number"));
    errorHandler();
  }

  // Check which specific operation we should perform.
  if (remainder.equals(SUBTOPIC_SERVO_BUFFER_PUSH))
  {
    onServoBufferPushMessage(servoIndex, payload);
  }
}

void setupMqtt()
{
  mqttClient.begin(mqttBrokerHostname, mqttBrokerPort, mqttEthernetClient);
  mqttClient.onMessage(onMqttMessage);
}

void connectMqtt()
{
  // Await the possible case where we're not linked.
  blockUntilEthernetLinked();

  // Connect to the MQTT broker.
  Serial.print(F("I: Connecting to MQTT broker"));
  do
  {
    Serial.print(F("."));
    delay(500);
  } while ((!mqttClient.connect(mqttClientIdentifier, mqttClientUsername, mqttClientPassword)));
  Serial.println(F("Connected!"));

  // Subscribe to the topics.
  mqttClient.subscribe("/nodes/arduino/servo/+/buffer/push");
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);

  // Start the serial port and wait for it to become steady.
  Serial.begin(SERIAL_BAUD);
  while (!Serial)
    ;

  // Perform the setup of the ethernet peripheral.
  setupEthernet();

  setupMqtt();

  connectMqtt();

  setupServos();
}

void loop()
{
  if (!mqttClient.loop())
  {
    connectMqtt();
  }

  for (ServoMod &servo : servos)
  {
    servo.loop();
  }
}
