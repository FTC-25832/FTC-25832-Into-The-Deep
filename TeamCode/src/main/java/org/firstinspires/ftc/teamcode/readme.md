### wireless code upload setup
connect to wifi then adb commands

`adb tcpip 5555`

`adb connect 192.168.43.1:5555`

faq https://www.reddit.com/r/FTC/comments/181l7i6/connecting_to_control_hub_wirelessly_with_adb/

detailed https://blog.jcole.us/2017/04/13/wireless-programming-for-ftc-robots/#appendix-usb-wifi-models

once adb established and connected to robot wifi in 'Program Ampersand Manage', build files will automatically upload


### links and stuff
- `192.168.43.1`
- FTC Dashboard at address::8080/dash 8030/dash  ---- this for roadrunner + canvas funcs
- FTControl at :8001 + :5801 can also limelight if want separate ----- this dash better for everything else
- Robot logs at :8080/logs
- Limelight if plugged in to computer at http://limelight.local:5801




# FTC Team Code Documentation

## System Architecture

### Core Systems
1. **Hardware Management**
   - `BulkReadManager`: Optimized hardware access with caching
   - `TimingConstants`: System-wide timing configuration
   - Performance monitoring and logging

2. **Vision System**
   - State machine for detection management
   - Multi-target tracking
   - Calibration and error recovery
   - Pipeline switching support

3. **Localization**
   - Three dead wheel odometry
   - IMU integration
   - Position filtering and validation
   - Real-time visualization

4. **Autonomous Control**
   - State-based navigation
   - Command sequencing
   - Path recording and replay
   - Validation and testing tools

### System Interactions
```
BulkReadManager
    ↓
    → MecanumDrive ← ThreeDeadWheelLocalizer
         ↓
         → AutoCommandSequence ← VisionStateManager
              ↓                      ↓
              → RobotVisualizer  ←  VisionPipelineManager
```

## Configuration

### Hardware Configuration
1. Motor Names:
   - `frontLeft`
   - `frontRight`
   - `backLeft`
   - `backRight`

2. Dead Wheel Encoders:
   - `leftEncoder`
   - `rightEncoder`
   - `centerEncoder`

3. Vision:
   - Camera name: `Webcam 1`
   - Resolution: 640x480
   - Frame rate: 30fps

### Performance Parameters
```java
public class TimingConstants {
    // Vision timeouts
    public static int VISION_DETECTION_TIMEOUT = 1000;
    public static int VISION_PIPELINE_SWITCH_DELAY = 50;
    
    // Loop timing thresholds
    public static int LOOP_TIME_WARNING = 50;  // ms
    public static int TARGET_LOOP_TIME = 7;    // ms
    
    // System validation
    public static int SYSTEM_CHECK_TIMEOUT = 500;
    public static int IMU_INIT_TIMEOUT = 2000;
}
```

## Setup Instructions

### 1. Development Environment
1. Install Android Studio
2. Clone repository
3. Configure FTC SDK
4. Build and deploy

### 2. Robot Configuration
1. Configure hardware in Robot Controller
2. Verify motor/sensor connections
3. Calibrate dead wheel odometry
4. Test vision system calibration

### 3. Pre-Match Validation
1. Run `TestSystemValidationOpMode`
2. Verify all systems pass checks
3. Test vision detection
4. Validate autonomous paths

## Testing Tools

### 1. Unit Tests
- `RobotStateTest`: Validate state transitions
- `BulkReadManagerTest`: Hardware access testing
- `VisionStateManagerTest`: Vision system validation

### 2. Integration Tests
- `RobotSystemsIntegrationTest`: Full system integration
- Path recording and comparison
- Performance monitoring

### 3. Test OpModes
- `TestSpeedOpMode`: Performance testing
- `TestVisionStateOpMode`: Vision system testing
- `TestAutoPathsOpMode`: Path validation
- `TestSystemValidationOpMode`: System checks

## Performance Monitoring

### Real-time Metrics
1. Loop timing statistics
2. Vision processing times
3. Position tracking accuracy
4. Command execution status

### Dashboard Integration
1. Real-time visualization
2. Path recording/replay
3. System state monitoring
4. Error reporting

## Error Handling

### Hardware Errors
1. Bulk read failures
2. Encoder disconnections
3. Motor stalls
4. Sensor timeouts

### Vision System
1. Pipeline errors
2. Camera disconnection
3. Frame processing failures
4. Detection loss recovery

### State Machine
1. Invalid transitions
2. Timeout handling
3. Error state recovery
4. Emergency stops

## Maintenance

### Code Style
1. Follow Google Java Style Guide
2. Use consistent naming conventions
3. Document public interfaces
4. Add unit tests for new features

### Performance Optimization
1. Monitor loop times
2. Use bulk reads where possible
3. Optimize vision processing
4. Cache hardware access

### Debug Tools
1. FTC Dashboard
2. Telemetry logging
3. State visualization
4. Path recording

### Common Issues
1. **Vision Detection Loss**
   - Check camera focus
   - Verify lighting conditions
   - Review pipeline parameters

2. **Position Tracking Drift**
   - Calibrate dead wheels
   - Check encoder connections
   - Verify wheel radius

3. **Loop Time Warnings**
   - Review bulk reads
   - Check vision processing
   - Monitor garbage collection

4. **State Transitions**
   - Verify state conditions
   - Check timeouts
   - Review error handling
