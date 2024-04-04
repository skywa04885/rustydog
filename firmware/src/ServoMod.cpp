#include <Arduino.h>

#include "ServoMod.hpp"

extern void errorHandler();

/// @brief Construct a new angle buffer.
SpeedServoBuffer::SpeedServoBuffer()
    : size(0),
      readIndex(0),
      writeIndex(0),
      drainCallback(nullptr),
      emptyCallback(nullptr)
{
}

void SpeedServoBuffer::onDrain(DrainCallback drainCallback)
{
    this->drainCallback = drainCallback;
}

void SpeedServoBuffer::onEmpty(EmptyCallback emptyCallback)
{
    this->emptyCallback = emptyCallback;
}

bool SpeedServoBuffer::push(SpeedServoTarget angle)
{
    if (this->size == SPEED_SERVO_BUFFER_SIZE)
    {
        return false;
    }

    this->angles[this->writeIndex] = angle;

    this->writeIndex = (this->writeIndex + 1) % SPEED_SERVO_BUFFER_SIZE;

    this->size += 1;

    return true;
}

bool SpeedServoBuffer::pop(SpeedServoTarget &angle)
{
    if (this->size == 0)
    {
        return false;
    }

    angle = this->angles[this->readIndex];

    this->readIndex = (this->readIndex + 1) % SPEED_SERVO_BUFFER_SIZE;

    this->size -= 1;

    if (this->size == 0 && this->emptyCallback != nullptr)
    {
        this->emptyCallback();
    }
    else if (this->size == SPEED_SERVO_BUFFER_HALF_SIZE)
    {
        this->drainCallback(SPEED_SERVO_BUFFER_SIZE - this->size);
    }

    return true;
}

bool SpeedServoBuffer::isEmpty()
{
    return this->size == 0;
}

ServoMod::ServoMod(int pin)
    : state(State::Uninitialized),
      nextState(State::Uninitialized),
      angle(0),
      pin(pin)
{
    this->stateEntry();
}

SpeedServoBuffer &ServoMod::getBuffer()
{
    return this->buffer;
}

bool ServoMod::setup()
{
    if (this->state != State::Uninitialized)
    {
        return false;
    }

    this->servo.attach(pin);
    this->servo.write(this->angle);

    this->transition(State::Idle);

    return true;
}

void ServoMod::loop()
{
    // Perform transition if needed.
    if (this->state != this->nextState)
    {
        this->stateExit();
        this->state = this->nextState;
        this->stateEntry();
    }

    // Perform the do of the state.
    this->stateDo();
}

bool ServoMod::transition(State state)
{
    if (state == this->state || state == this->nextState)
    {
        return false;
    }

    this->nextState = state;

    return true;
}

void ServoMod::stateEntry()
{
    switch (this->state)
    {
    case State::Adjusting:
        this->adjustingEntry();
        break;
    default:
        break;
    }
}

void ServoMod::stateDo()
{
    switch (this->state)
    {
    case State::Adjusting:
        this->adjustingDo();
        break;
    case State::Idle:
        this->idleDo();
        break;
    default:
        break;
    }
}

void ServoMod::stateExit()
{
    switch (this->state)
    {
    case State::Adjusting:
        this->adjustingExit();
        break;
    default:
        break;
    }
}

void ServoMod::idleDo()
{
    // Don't do anything is the angle buffer is empty.
    if (this->buffer.isEmpty())
    {
        return;
    }

    // Transition to the adjusting state.
    this->transition(State::Adjusting);
}

void ServoMod::adjustingEntry()
{
    // Pop the angle of the buffer, and make sure it exists.
    if (!this->buffer.pop(this->adjustingState.target))
    {
        Serial.println(F("E: Servo adjusting state entered, but no angle to pop!"));
        errorHandler();
    }

    // Enable first iteration because we want to immediately write.
    this->adjustingState.firstIter = true;

    // Compute the interval millis.
    this->adjustingState.intervalMillis = 1000UL / this->adjustingState.target.speed;
}

void ServoMod::adjustingDo()
{
    // Check whether we should perform any update.
    if (this->adjustingState.firstIter)
    {
        this->adjustingState.firstIter = false;
    }
    else if (millis() - this->adjustingState.lastMillis < this->adjustingState.intervalMillis)
    {
        return;
    }

    // Adjust the current angle.
    if (this->adjustingState.target.angle > this->angle)
    { // Need to increase to get to target angle.
        this->angle += 1;
    }
    else if (this->adjustingState.target.angle < this->angle)
    { // Need to decrease to get to target angle.
        this->angle -= 1;
    }
    else
    { // At the target angle.
        this->transition(State::Idle);
        return;
    }

    // Write the new angle.
    this->servo.write(this->angle);

    // Set the last millis to know when to perform the next update.
    this->adjustingState.lastMillis = millis();
}

void ServoMod::adjustingExit() {}