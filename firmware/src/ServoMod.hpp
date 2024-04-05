#pragma once

#include <Servo.h>

#define SPEED_SERVO_BUFFER_SIZE 20
#define SPEED_SERVO_BUFFER_HALF_SIZE 10

struct SpeedServoTarget
{
public:
    int angle;
    int speed;
};

class SpeedServoBuffer
{
public:
    using DrainCallback = void (*)(int);
    using EmptyCallback = void (*)();

private:
    SpeedServoTarget angles[SPEED_SERVO_BUFFER_SIZE];
    int size, readIndex, writeIndex;
    DrainCallback drainCallback;
    EmptyCallback emptyCallback;

public:
    /// @brief Construct a new buffer.
    SpeedServoBuffer();

    /// @brief Set the callback that should be called when the buffer is drained.
    /// @param drainCallback The callback to call.
    void onDrain(DrainCallback drainCallback);

    /// @brief Set the callback to be called when the buffer is empty.
    /// @param emptyCallback The callback to call when the buffer is empty.
    void onEmpty(EmptyCallback emptyCallback);

    /// @brief Push a new target into the buffer.
    /// @param angle The target to push.
    /// @return Whether or not the angle was pushed.
    bool push(SpeedServoTarget angle);

    /// @brief Pop an target from the buffer.
    /// @param angle a reference to where the popped target should be stored.
    /// @return Whether or not there was an target popped.
    bool pop(SpeedServoTarget &angle);

    /// @brief Check if the buffer is empty.
    /// @return Whether or not the buffer is empty.
    bool isEmpty();
};

class ServoMod
{
public:
    /// @brief The state of the servo.
    enum class State
    {
        Uninitialized = 0,
        Idle = 1,
        Adjusting = 2,
    };

private:
    /// @brief Represents the state of the adjusting mode.
    struct AdjustingState
    {
    public:
        SpeedServoTarget target;
        unsigned long lastMillis;
        bool firstIter;
        unsigned long intervalMillis;
    };

private:
    SpeedServoBuffer buffer;
    AdjustingState adjustingState;
    Servo servo;
    State state, nextState;
    int angle, pin;

public:
    /// @brief Construct a new servo on the given pin.
    /// @param pin The pin.
    ServoMod(int pin);

    /// @brief Get the buffer.
    /// @return A reference to the buffer.
    SpeedServoBuffer &getBuffer();

    /// @brief Perform the setup of the servo.
    /// @return
    bool setup();

    /// @brief Perform the loop of the servo.
    void loop();

    /// @brief Trigger a transition to the given state.
    /// @param state The state to transition to.
    /// @return Whether or not the given state will be transitioned to.
    bool transition(State state);

    /// @brief Get the current angle.
    /// @return the angle.
    int getAngle();

private:
    /// @brief Enter the current state.
    void stateEntry();

    /// @brief Do the current state.
    void stateDo();

    /// @brief Exit the current state.
    void stateExit();

    /// @brief Do the idle state.
    void idleDo();

    /// @brief Enter the adjusting state.
    void adjustingEntry();

    /// @brief Do the adjusting state.
    void adjustingDo();

    /// @brief Exit the adjusting state.
    void adjustingExit();
};
