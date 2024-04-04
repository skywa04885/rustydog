# MQTT

In the topics, `{n}` will be used to indicate that the servo number must be given there.

## Instructions

### Push angles to path.

Topic: `/nodes/arduino/servo/{n}/buffer/push`

```json
{
    "angles": [
        120.0,
        40.0,
        20.0,
    ]
}
```

### Set speed.

Topic: `/nodes/arduino/servo/{n}/speed/set`

```json
{
    "speed": 40.0
}
```

## Events

### Path buffer drain.

Topic: `/nodes/arduino/servo/{n}/buffer/drain`

```json
{
    "need": 10
}
```

### Path buffer empty.

Topic: `/nodes/arduino/servo/{n}/buffer/empty`

