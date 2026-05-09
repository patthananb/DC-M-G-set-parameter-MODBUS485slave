#pragma once

namespace config { namespace dcmotormeasurement {

// Convert a raw ADC voltage to a motor voltage using the piecewise-linear
// mapping defined in Config.h:
//   adcV <  ZERO_CROSS_V : linear from MIN_SENSOR_V (-MOTOR_V_MAX) to
//                          ZERO_CROSS_V (0)
//   adcV >= ZERO_CROSS_V : linear from ZERO_CROSS_V (0) to
//                          MAX_SENSOR_V (+MOTOR_V_MAX)
// Result is clamped to ±MOTOR_V_MAX.
float adcToMotor(float adcV);

}  // namespace dcmotormeasurement
}  // namespace config
