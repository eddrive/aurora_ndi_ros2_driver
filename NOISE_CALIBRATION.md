# Aurora Noise Calibration Guide

## Overview

This guide explains how to calibrate the noise characteristics of your Aurora NDI sensor(s) to optimize the Kalman filter parameters for your specific setup and environment.

## What is Noise Calibration?

The Aurora NDI tracking system has inherent measurement noise due to:
- Electromagnetic field variations
- Sensor electronics noise
- Environmental interference (metal objects, other EM sources)
- Quantization errors

By measuring this noise with the sensor stationary, we can:
1. **Characterize the baseline noise** (position RMS, orientation drift)
2. **Compute optimal Kalman filter parameters** (measurement noise covariance)
3. **Detect environmental issues** (unusual noise patterns, interference)

## Prerequisites

### Hardware Setup
1. **Mount the sensor rigidly** - Use a stable fixture or clamp
2. **Position in tracking volume** - Ensure good visibility and typical working distance
3. **Minimize interference** - Remove unnecessary metal objects
4. **Stable environment** - No vibrations, air currents, or movement

### Software Requirements
- Aurora driver properly configured (`aurora_config.yaml`)
- Sensor publishing data at nominal rate (40 Hz)
- ROS 2 workspace built successfully

## Step-by-Step Calibration Procedure

### 1. Prepare the Sensor

```bash
# Test that the Aurora driver works
ros2 launch aurora_ndi_ros2_driver aurora_pub.launch.py

# Verify data is being published (in another terminal)
ros2 topic hz /aurora_data_sensor0

# Expected output: ~40 Hz
# Press Ctrl+C to stop
```

### 2. Mount Sensor Stationary

**CRITICAL:** The sensor must be **completely stationary** during calibration!

- Use a rigid mount (e.g., clamp to table)
- Ensure no vibrations or movement
- Keep distance from metal objects
- Typical working distance: 30-50 cm from field generator

### 3. Run Calibration

```bash
# Basic calibration (1000 samples, ~25 seconds)
ros2 launch aurora_ndi_ros2_driver aurora_noise_calibration.launch.py

# Quick test (500 samples, ~12 seconds)
ros2 launch aurora_ndi_ros2_driver aurora_noise_calibration.launch.py max_samples:=500

# Thorough calibration (2000 samples, ~50 seconds)
ros2 launch aurora_ndi_ros2_driver aurora_noise_calibration.launch.py max_samples:=2000

# Custom output file
ros2 launch aurora_ndi_ros2_driver aurora_noise_calibration.launch.py \
    output_file:=$HOME/my_calibration_results.yaml
```

### 4. Interpret Results

The calibration will print statistics like this:

```
============================================
  NOISE ESTIMATION RESULTS
============================================

--- Sensor 0 (Topic: /aurora_data_sensor0) ---
Samples: 1000 valid / 1000 total (0.0% rejected)
Duration: 25.00 seconds

Position Noise (mm):
  Mean:    [123.456, -78.901, 234.567]
  Std Dev: [0.523, 0.489, 0.612]
  Range X: [122.341, 124.782] (2.441 mm)
  Range Y: [-79.876, -77.932] (1.944 mm)
  Range Z: [232.876, 236.123] (3.247 mm)
  RMS deviation: 0.891 mm

Orientation Noise:
  Mean Quaternion: [0.0123, -0.0456, 0.7891, 0.6123]
  Std Dev:         [0.0034, 0.0029, 0.0041, 0.0038]

Error Statistics (mm):
  Mean:    0.842
  Std Dev: 0.178
  Max:     1.523

>>> RECOMMENDED KALMAN FILTER PARAMETERS <<<
  measurement_noise_position: 0.3746  # R matrix (position variance)
  measurement_noise_orientation: 0.000017  # R matrix (orientation variance)
============================================
```

### 5. Understanding the Results

#### Position Statistics
- **Mean**: Average position (reference point, not important)
- **Std Dev**: Noise magnitude per axis (typically 0.3-1.0 mm)
- **RMS deviation**: Overall positional noise (typical: 0.5-1.5 mm)
- **Range**: Total drift during collection (should be small)

#### Orientation Statistics
- **Mean Quaternion**: Average orientation (reference, not important)
- **Std Dev**: Quaternion component noise (typical: 0.001-0.01)

#### Error Statistics
- **Mean**: Average Aurora internal error estimate (typical: 0.5-2.0 mm)
- **Std Dev**: Variability of error estimate
- **Max**: Worst-case error during collection

#### Recommended Parameters
These values should be copied to `kalman_filter_config.yaml`:
- `measurement_noise_position`: Position variance (std_dev²)
- `measurement_noise_orientation`: Orientation variance (std_dev²)

### 6. Save Results

Results are automatically saved to `noise_calibration_results.yaml`:

```yaml
noise_calibration:
  num_sensors: 1

  sensor_0:
    topic: "/aurora_data_sensor0"
    samples:
      valid: 1000
      total: 1000
      rejected: 0
    position:
      mean: [123.456, -78.901, 234.567]
      std: [0.523, 0.489, 0.612]
      rms: 0.891
    orientation:
      mean: [0.0123, -0.0456, 0.7891, 0.6123]
      std: [0.0034, 0.0029, 0.0041, 0.0038]
    error:
      mean: 0.842
      std: 0.178
      max: 1.523
    recommended_kalman_params:
      measurement_noise_position: 0.3746
      measurement_noise_orientation: 0.000017
```

## Troubleshooting

### Problem: High Position RMS (>2.0 mm)

**Possible causes:**
- Sensor is moving (vibrations, not properly fixed)
- Metal interference nearby
- Poor sensor visibility (occlusion)
- Suboptimal working distance

**Solutions:**
- Improve mounting rigidity
- Remove metal objects
- Adjust sensor position
- Check sensor visibility indicator

### Problem: Many Rejected Samples

**Possible causes:**
- Sensor visibility issues
- `max_error_threshold` too strict
- Intermittent tracking loss

**Solutions:**
- Adjust sensor position for better visibility
- Increase `max_error_threshold` in config (default: 5.0 mm)
- Check field generator calibration

### Problem: Uneven Axis Noise

Example: X=0.5mm, Y=0.5mm, Z=2.0mm

**Possible causes:**
- Field distortion (metal objects)
- Sensor orientation effects
- Suboptimal sensor placement

**Solutions:**
- Rotate sensor to different orientation
- Move sensor to different location
- Check for metal interference in Z direction

### Problem: High Orientation Noise (>0.02)

**Possible causes:**
- Sensor movement during collection
- Cable strain causing rotation
- Poor tracking quality

**Solutions:**
- Ensure cable is not under tension
- Improve sensor mounting
- Verify sensor is within optimal tracking volume

## Advanced Usage

### Multi-Sensor Calibration

For multiple sensors, edit `config/noise_estimator_config.yaml`:

```yaml
num_sensors: 2
topic_names:
  - "/aurora_data_sensor0"
  - "/aurora_data_sensor1"
```

Then run calibration - each sensor will be calibrated independently.

### Continuous Monitoring

For diagnostics, you can run the estimator in continuous mode:

```yaml
# In noise_estimator_config.yaml
batch_mode: false
max_samples: 0
collection_duration_sec: 0.0
publish_rate_hz: 5.0  # Print stats every 0.2 seconds
```

This is useful for:
- Monitoring noise during system operation
- Detecting environmental changes
- Verifying Kalman filter performance

### Environment-Specific Calibration

For colonoscopy applications, consider calibrating in the actual operating environment:

1. **In-situ calibration** - Calibrate with sensor near surgical field
2. **With interference** - Include typical EM sources (electrocautery, monitors)
3. **Multiple positions** - Calibrate at different distances/orientations

## Best Practices

### When to Calibrate

- **Initial setup** - Always calibrate new hardware
- **Environment change** - New room, different equipment
- **After maintenance** - Field generator recalibration
- **Periodic validation** - Monthly or quarterly checks
- **Performance issues** - If tracking seems noisier than expected

### Calibration Quality

Good calibration indicators:
- ✅ Position RMS: 0.5-1.5 mm
- ✅ Orientation std: 0.001-0.01
- ✅ Low rejection rate: <5%
- ✅ Consistent across runs

Poor calibration indicators:
- ⚠️ Position RMS: >2.0 mm
- ⚠️ High rejection rate: >20%
- ⚠️ Inconsistent results between runs

### Documentation

Keep records of your calibrations:
```bash
# Date-stamped calibration files
ros2 launch aurora_ndi_ros2_driver aurora_noise_calibration.launch.py \
    output_file:=calibration_$(date +%Y%m%d_%H%M%S).yaml
```

## Next Steps

After successful calibration:

1. **Copy parameters** to `kalman_filter_config.yaml`
2. **Test Kalman filter** with real data
3. **Tune process noise** based on application dynamics
4. **Validate performance** in target application

See `KALMAN_FILTER.md` for next steps in configuring and testing the filter.

## Reference

### Typical Noise Values

| Environment | Position RMS | Orientation Std | Notes |
|------------|--------------|-----------------|--------|
| Ideal lab | 0.5-0.8 mm | 0.001-0.005 | Minimal interference |
| Clinical OR | 0.8-1.5 mm | 0.005-0.010 | Some EM interference |
| Poor setup | >2.0 mm | >0.015 | Metal/interference issues |

### Configuration Parameters

All parameters in `config/noise_estimator_config.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `min_samples` | 100 | Minimum samples for statistics |
| `max_samples` | 1000 | Samples to collect in batch mode |
| `collection_duration_sec` | 30.0 | Maximum collection time |
| `batch_mode` | true | Exit after collection |
| `require_visible` | true | Only use visible samples |
| `max_error_threshold` | 5.0 | Reject high-error samples (mm) |
| `save_to_file` | true | Save results to YAML |
| `verbose` | true | Print detailed statistics |

## Support

For issues or questions:
- Check troubleshooting section above
- Review Aurora NDI documentation
- Verify hardware setup and connections
- Test with minimal configuration (single sensor)
