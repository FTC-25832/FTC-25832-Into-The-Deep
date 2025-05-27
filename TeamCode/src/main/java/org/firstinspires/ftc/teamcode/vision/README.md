# Vision System Architecture

## Overview
The vision system provides a robust framework for AprilTag detection, tracking, and robot alignment. It uses a state machine architecture combined with a command pattern for flexible and reusable vision-based behaviors.

## Core Components

### 1. Vision State Management
- `VisionState` - Enum defining possible vision processing states
- `VisionStateContext` - Interface for state transitions
- `VisionStateManager` - Manages state transitions and timing
- `VisionPipelineManager` - Handles camera pipeline switching

### 2. AprilTag Detection
- `AprilTagDetectionPipeline` - OpenCV pipeline for tag detection and pose estimation
- Uses EasyOpenCV and April Tags libraries
- Supports visualization and debug overlays

### 3. Command System
- `AprilTagCommand` - Base command for tag-based operations
- `AprilTagCommandGroup` - Chains multiple commands for complex behaviors
- Factory methods for common sequences (search, align, dock)

## Configuration and Constants
- `VisionConstants` - Camera parameters and tag positions
- Configurable parameters:
  - Camera mount position and orientation
  - AprilTag detection thresholds
  - Field tag layout

## Usage Examples

### 1. Basic Tag Detection
```java
// Initialize pipeline
AprilTagDetectionPipeline pipeline = new AprilTagDetectionPipeline();
camera.setPipeline(pipeline);

// Get detections
ArrayList<AprilTagDetection> detections = pipeline.getLatestDetections();
```

### 2. Command-Based Alignment
```java
// Create alignment sequence
AprilTagCommandGroup alignSequence = AprilTagCommandGroup.createAlignmentSequence(
    pipeline,
    targetTagId,
    12.0  // inches
);

// Execute sequence
alignSequence.initialize();
while (!alignSequence.isFinished()) {
    alignSequence.execute();
}
```

### 3. Autonomous Docking
```java
// Create docking sequence
AprilTagCommandGroup dockSequence = AprilTagCommandGroup.createDockingSequence(
    pipeline,
    targetTagId
);

// Execute with state machine
dockSequence.initialize();
while (!dockSequence.isFinished()) {
    dockSequence.execute();
    
    // Optional: Check progress
    double progress = dockSequence.getProgress();
    boolean tagsFound = dockSequence.wereTagsFound();
}
```

## Test OpModes

### 1. TestAprilTagOpMode
- Basic tag detection and visualization
- Camera stream testing
- Tag ID verification

### 2. TestAprilTagCommandOpMode
- Command system validation
- Interactive testing of search/align/dock behaviors
- Progress monitoring and telemetry

### 3. AprilTagAlignmentOpMode
- PID-controlled alignment to tags
- Distance and angle tuning
- Error handling and recovery

## Tuning Guidelines

### Detection Parameters
1. Adjust `DECIMATION_HIGH` and `DECIMATION_LOW` for processing speed vs. accuracy
2. Tune `THRESHOLD_HIGH` and `THRESHOLD_LOW` for detection reliability
3. Set `MIN_CONFIDENCE` to filter false positives

### Alignment Parameters
1. Modify `SPEED_GAIN`, `STRAFE_GAIN`, and `TURN_GAIN` for movement control
2. Adjust distance and heading tolerances
3. Tune timeout thresholds for state transitions

## Field Setup
1. Place AprilTags at known positions defined in `VisionConstants`
2. Calibrate camera mount parameters
3. Verify tag detection ranges
4. Test alignment accuracy at various distances

## Best Practices
1. Initialize vision system early in OpMode
2. Monitor processing times with LoopTimer
3. Handle camera errors gracefully
4. Clean up resources in stop()
5. Use command groups for complex sequences
6. Add appropriate telemetry for debugging

## Dependencies
- EasyOpenCV
- April Tags library
- RoadRunner (for pose estimation)
