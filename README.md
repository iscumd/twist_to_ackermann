# twist_to_ackermann
From package '[twist_to_ackermann]()'
# File
`./src/twist_to_ackermann.cpp`

## Summary 
 Translates twist commands to ackermann drive commands, to be used in car-like robots.

## Topics

### Publishes
- `/ack_vel`: A stamped or unstamped ackermann drive command resulting from the translation.

### Subscribes
- `/nav_vel`: The stamped or unstamped twist message to convert.

## Params
- `use_stamps`: If true, TwistStamped and AckermannDriveStamped messages will be used. defualt: false
- `wheelbase`: The wheelbase of your vehicle in meters (float). Required, but defaults to 1.0

