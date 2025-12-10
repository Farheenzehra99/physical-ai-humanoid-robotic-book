# Module 2, Chapter 5, Page 15: Simulating Sensors: LiDAR, Cameras, IMUs

**Book**: Physical AI & Humanoid Robotics — A Practical Guide
**Author**: Syeda Farheen Zehra
**Module 2**: The Digital Twin (Gazebo & Unity)
**Chapter 5**: Unity for High-Fidelity Rendering

---

## Introduction: Giving Your Robot "Senses"

Imagine navigating a dark room blindfolded. You'd struggle to:
- See where objects are
- Know which way is "up"
- Detect obstacles ahead
- Identify people or objects

**Robots face the same challenge.** Without sensors, your beautifully designed humanoid is blind, deaf, and disoriented.

**Sensors are the robot's senses**:
- **Cameras** = Eyes (see the world)
- **LiDAR** = Echo-location (measure distances like bats)
- **IMU** = Inner ear (know orientation and acceleration)

In Unity, you can simulate these sensors with photorealistic accuracy—generating training data, testing perception algorithms, and validating navigation without touching real hardware.

---

## Why Simulate Sensors?

**Real-World Problem**: Sensors are expensive and fragile
- **LiDAR unit**: $1,000-$75,000 (Velodyne VLP-16 to HDL-64E)
- **Depth camera**: $100-$500 (Intel RealSense, Kinect)
- **IMU**: $50-$5,000 (consumer to navigation-grade)

**If you crash** during testing:
- Hardware breaks → $1,000s replacement cost
- Development stops → weeks of delay
- Limited scenarios → can't test extreme conditions

**Unity Solution**: Simulate sensors perfectly
- **Test 1,000 scenarios overnight** (no hardware risk)
- **Generate synthetic training data** (50,000 images in hours)
- **Extreme conditions** (darkness, fog, rain—difficult to create physically)
- **Iterate rapidly** (change sensor placement in seconds)

**Example**: Training object detection
- Real-world: Take 10,000 photos manually (weeks of work)
- Unity: Auto-generate 10,000 labeled images (overnight)

---

## The Three Essential Sensors

### Sensor Comparison Table

| Sensor | Real-World Analogy | What It Measures | Use Cases |
|--------|-------------------|------------------|-----------|
| **Camera** | Human eyes | RGB images, colors, textures | Object detection, navigation, face recognition |
| **Depth Camera** | Eyes with depth perception | Distance to each pixel | 3D mapping, obstacle avoidance, manipulation |
| **LiDAR** | Bat echo-location | Distances in 360° | SLAM, localization, mapping |
| **IMU** | Inner ear balance | Orientation, acceleration, angular velocity | Balance, tilt detection, odometry |

---

## 1. Camera Sensors: The Robot's Eyes

### What Cameras Provide

**RGB Cameras** capture what humans see:
- Colors (red, green, blue)
- Textures (smooth, rough, patterned)
- Shapes and edges
- Lighting conditions

**Real-World Uses**:
- **Object detection**: "Is that a coffee cup or a water bottle?"
- **Face recognition**: "Is this person authorized?"
- **Navigation**: "Follow the yellow line on the floor"
- **Manipulation**: "Align gripper with handle"

### Simulating Cameras in Unity

Unity cameras simulate real cameras with incredible detail:
- **Lens properties**: Focal length, aperture, depth of field
- **Sensor specs**: Resolution, ISO, shutter speed
- **Image effects**: Motion blur, chromatic aberration, lens distortion
- **Realistic lighting**: Ray-traced reflections, global illumination

**Setup: Adding an RGB Camera to Robot**

1. **Create Camera GameObject**:
   - Hierarchy → Right-click → Create Empty → Name: "RobotCamera"
   - Position: Attach to robot's head link
     - Transform → Position: (0, 0.15, 0.05) relative to head
     - Transform → Rotation: (0, 0, 0) looking forward

2. **Add Camera Component**:
   - Inspector → Add Component → Camera
   - Settings:
     - Field of View: 60° (human-like)
     - Clipping Planes: Near 0.1, Far 100
     - Target Texture: Create RenderTexture (512×512 or 1920×1080)

3. **Configure for Robotics**:
   - Clear Flags: Skybox
   - Culling Mask: Everything
   - Depth: 0 (higher = renders on top)
   - Rendering Path: Use Graphics Settings

**C# Script: Capture Camera Images**

```csharp
using UnityEngine;
using System.IO;

public class RobotCameraCapture : MonoBehaviour
{
    public Camera robotCamera;
    public int imageWidth = 640;
    public int imageHeight = 480;
    public string savePath = "Assets/CapturedImages/";

    private int imageCounter = 0;

    void Start()
    {
        // Create save directory if it doesn't exist
        if (!Directory.Exists(savePath))
        {
            Directory.CreateDirectory(savePath);
        }
    }

    void Update()
    {
        // Press C to capture image
        if (Input.GetKeyDown(KeyCode.C))
        {
            CaptureImage();
        }
    }

    void CaptureImage()
    {
        // Create RenderTexture
        RenderTexture rt = new RenderTexture(imageWidth, imageHeight, 24);
        robotCamera.targetTexture = rt;

        // Render camera to texture
        robotCamera.Render();

        // Read pixels from RenderTexture
        RenderTexture.active = rt;
        Texture2D image = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
        image.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        image.Apply();

        // Save to disk
        byte[] bytes = image.EncodeToPNG();
        string filename = savePath + "image_" + imageCounter.ToString("D4") + ".png";
        File.WriteAllBytes(filename, bytes);

        Debug.Log("Captured: " + filename);

        // Cleanup
        robotCamera.targetTexture = null;
        RenderTexture.active = null;
        Destroy(rt);

        imageCounter++;
    }
}
```

**Usage**:
- Attach script to robot or camera GameObject
- Assign `robotCamera` in Inspector
- Play mode → Press **C** to capture images
- Images saved to `Assets/CapturedImages/`

**Use Case: Object Detection Training**
- Place objects in scene (cups, boxes, chairs)
- Move robot around (vary angles, lighting, distances)
- Capture 1,000s of images automatically
- Each image auto-labeled (Unity knows 3D positions)
- Train YOLO, Faster R-CNN, or other detectors

---

## 2. Depth Cameras: Seeing in 3D

### What Depth Cameras Provide

**Depth cameras** measure distance to every pixel:
- RGB image: (Red, Green, Blue) values
- **+ Depth map**: Distance in meters to each pixel

**Example Output**:
```
Pixel (320, 240):
  RGB: (150, 200, 100) — greenish color
  Depth: 2.5 meters — object is 2.5m away
```

**Real-World Uses**:
- **Obstacle avoidance**: "Wall 0.3m ahead, stop!"
- **3D mapping**: Build point cloud of environment
- **Manipulation**: "Gripper 5cm from object"
- **Gesture recognition**: Track hand movements in 3D

### Simulating Depth Cameras in Unity

Unity provides depth through **shader-based rendering**:
- Each pixel's depth extracted from z-buffer
- Output as grayscale image (closer = darker)
- Or point cloud (x, y, z coordinates per pixel)

**Setup: Adding Depth Camera**

1. **Create Second Camera** (same as RGB camera)
   - Duplicate RobotCamera → Rename "DepthCamera"
   - Same position and rotation

2. **Attach Depth Shader Script**:

```csharp
using UnityEngine;

public class DepthCamera : MonoBehaviour
{
    public Camera depthCamera;
    public RenderTexture depthTexture;

    void Start()
    {
        // Create RenderTexture for depth
        depthTexture = new RenderTexture(640, 480, 24, RenderTextureFormat.Depth);
        depthCamera.targetTexture = depthTexture;

        // Set camera to render depth
        depthCamera.depthTextureMode = DepthTextureMode.Depth;
    }

    void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        // Apply depth visualization shader
        Graphics.Blit(source, destination, depthMaterial);
    }

    // Shader converts depth buffer to visible grayscale
    private Material depthMaterial;

    void Awake()
    {
        // Create material with depth shader
        Shader depthShader = Shader.Find("Custom/DepthGrayscale");
        depthMaterial = new Material(depthShader);
    }
}
```

**Depth Visualization Shader** (create as `DepthGrayscale.shader`):

```glsl
Shader "Custom/DepthGrayscale"
{
    Properties
    {
        _MainTex ("Texture", 2D) = "white" {}
    }
    SubShader
    {
        Pass
        {
            CGPROGRAM
            #pragma vertex vert_img
            #pragma fragment frag
            #include "UnityCG.cginc"

            sampler2D _CameraDepthTexture;

            float4 frag(v2f_img i) : SV_Target
            {
                // Sample depth buffer
                float depth = SAMPLE_DEPTH_TEXTURE(_CameraDepthTexture, i.uv);

                // Convert to linear 0-1 range
                depth = Linear01Depth(depth);

                // Return as grayscale (closer = darker)
                return float4(depth, depth, depth, 1);
            }
            ENDCG
        }
    }
}
```

**Output**:
- **Black pixels**: Close objects (0-1m)
- **Gray pixels**: Medium distance (1-5m)
- **White pixels**: Far objects (5-100m)

**Use Case: Point Cloud Generation**
- Depth camera captures distances
- For each pixel:
  - RGB: Color from camera
  - Depth: Distance from depth map
  - 3D position: `(x, y, z) = camera_position + direction * depth`
- Result: Colored point cloud (for SLAM, mapping)

---

## 3. LiDAR: Laser Distance Scanning

### What LiDAR Provides

**LiDAR** (Light Detection and Ranging) shoots laser beams in 360°:
- Measures time for laser to bounce back
- Calculates distance: `distance = (speed_of_light * time) / 2`
- Produces **point cloud**: 3D coordinates of surfaces

**Typical Specs**:
- **Horizontal**: 360° (full rotation)
- **Vertical**: 16-128 channels (layers)
- **Range**: 0.5m to 100m+ (automotive LiDAR)
- **Frequency**: 10-20 Hz (10-20 scans/second)

**Real-World Uses**:
- **SLAM**: Simultaneous Localization and Mapping
- **Autonomous driving**: Detect cars, pedestrians, obstacles
- **Warehouse robots**: Navigate narrow aisles
- **3D scanning**: Create digital twins of buildings

### Simulating LiDAR in Unity

Unity simulates LiDAR using **raycasting**:
- Cast rays in all directions from sensor position
- Detect what each ray hits
- Measure distance to hit point
- Output as point cloud or range array

**Setup: Adding LiDAR Sensor**

```csharp
using UnityEngine;
using System.Collections.Generic;

public class LiDARSensor : MonoBehaviour
{
    [Header("LiDAR Configuration")]
    public int horizontalRays = 360;    // 360 rays = 1° resolution
    public int verticalChannels = 16;   // 16-layer LiDAR
    public float maxRange = 100f;       // 100m range
    public float minRange = 0.5f;       // 0.5m minimum
    public float verticalFOV = 30f;     // ±15° vertical

    [Header("Visualization")]
    public bool drawDebugRays = true;
    public Color hitColor = Color.green;
    public Color missColor = Color.red;

    private List<Vector3> pointCloud = new List<Vector3>();

    void Update()
    {
        ScanEnvironment();
    }

    void ScanEnvironment()
    {
        pointCloud.Clear();

        // Vertical angle step
        float verticalStep = verticalFOV / (verticalChannels - 1);
        float verticalStart = -verticalFOV / 2;

        for (int v = 0; v < verticalChannels; v++)
        {
            float verticalAngle = verticalStart + (v * verticalStep);

            for (int h = 0; h < horizontalRays; h++)
            {
                // Horizontal angle
                float horizontalAngle = (360f / horizontalRays) * h;

                // Calculate ray direction
                Vector3 direction = Quaternion.Euler(verticalAngle, horizontalAngle, 0) * transform.forward;

                // Cast ray
                RaycastHit hit;
                if (Physics.Raycast(transform.position, direction, out hit, maxRange))
                {
                    if (hit.distance >= minRange)
                    {
                        // Valid hit
                        pointCloud.Add(hit.point);

                        // Debug visualization
                        if (drawDebugRays)
                        {
                            Debug.DrawLine(transform.position, hit.point, hitColor);
                        }
                    }
                }
                else
                {
                    // No hit (ray went beyond maxRange)
                    if (drawDebugRays)
                    {
                        Debug.DrawRay(transform.position, direction * maxRange, missColor);
                    }
                }
            }
        }

        // Point cloud now contains all detected points
        // Can be published to ROS as sensor_msgs/PointCloud2
    }

    // Get latest point cloud
    public List<Vector3> GetPointCloud()
    {
        return new List<Vector3>(pointCloud);
    }

    void OnDrawGizmos()
    {
        // Visualize point cloud in Scene View
        if (pointCloud != null)
        {
            Gizmos.color = Color.cyan;
            foreach (Vector3 point in pointCloud)
            {
                Gizmos.DrawSphere(point, 0.02f);
            }
        }
    }
}
```

**Usage**:
- Attach to robot GameObject at sensor mounting position
- Configure:
  - `horizontalRays`: 360 for 1° resolution (more = denser, slower)
  - `verticalChannels`: 16, 32, 64, 128 (common LiDAR types)
  - `maxRange`: 100m (adjust for sensor type)
  - `drawDebugRays`: Enable to see rays in Scene View

**Visualization**:
- Green lines: Rays that hit objects
- Red lines: Rays that missed (open space)
- Cyan spheres: Point cloud points

**Performance Note**:
- 360 rays × 16 channels = **5,760 raycasts per frame**
- At 60 FPS: 345,600 raycasts/second
- Use Physics Layer Masks to ignore irrelevant objects (UI, effects)

**Optimization**:
```csharp
// Only scan objects on specific layers
public LayerMask scanLayers;

// In ScanEnvironment():
if (Physics.Raycast(transform.position, direction, out hit, maxRange, scanLayers))
{
    // ...
}
```

**Use Case: SLAM (Simultaneous Localization and Mapping)**
- LiDAR scans environment (walls, furniture, obstacles)
- Point cloud sent to SLAM algorithm (gmapping, cartographer)
- Robot builds 2D/3D map while tracking its position
- Enables autonomous navigation in unknown environments

---

## 4. IMU: The Robot's Balance Sensor

### What IMUs Provide

**IMU** (Inertial Measurement Unit) combines three sensors:

1. **Accelerometer**: Measures linear acceleration
   - 3 axes: X, Y, Z
   - Units: m/s²
   - Detects: Movement, tilt, vibration

2. **Gyroscope**: Measures angular velocity
   - 3 axes: Roll, Pitch, Yaw
   - Units: rad/s or °/s
   - Detects: Rotation, turning

3. **Magnetometer** (optional): Measures magnetic field
   - 3 axes: X, Y, Z
   - Units: Gauss or Tesla
   - Detects: Compass heading (North)

**Real-World Uses**:
- **Balance**: "Am I tilted? Adjust stance!"
- **Odometry**: Estimate position from acceleration
- **Fall detection**: "Acceleration spike → falling!"
- **Orientation tracking**: "Which way is up?"

### Simulating IMU in Unity

Unity's **Rigidbody** provides all IMU data:
- `rigidbody.velocity` → Linear velocity (integrate for position)
- `rigidbody.angularVelocity` → Gyroscope data
- Gravity + movement → Accelerometer data

**Setup: IMU Sensor Script**

```csharp
using UnityEngine;

public class IMUSensor : MonoBehaviour
{
    private Rigidbody rb;

    [Header("IMU Data (Read-Only)")]
    public Vector3 acceleration;      // m/s²
    public Vector3 angularVelocity;   // rad/s
    public Vector3 orientation;       // Euler angles (degrees)
    public Vector3 magneticField;     // Gauss (simulated)

    [Header("Noise Simulation")]
    public bool addNoise = true;
    public float accelNoise = 0.01f;  // ±0.01 m/s²
    public float gyroNoise = 0.001f;  // ±0.001 rad/s

    private Vector3 previousVelocity;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            Debug.LogError("IMU requires Rigidbody component!");
        }
        previousVelocity = rb.velocity;
    }

    void FixedUpdate()
    {
        // 1. ACCELEROMETER: Linear acceleration
        Vector3 currentVelocity = rb.velocity;
        Vector3 accel = (currentVelocity - previousVelocity) / Time.fixedDeltaTime;

        // Include gravity (real accelerometers measure gravity + motion)
        accel += Physics.gravity;

        // Transform to sensor's local frame
        acceleration = transform.InverseTransformDirection(accel);

        // Add realistic noise
        if (addNoise)
        {
            acceleration += new Vector3(
                Random.Range(-accelNoise, accelNoise),
                Random.Range(-accelNoise, accelNoise),
                Random.Range(-accelNoise, accelNoise)
            );
        }

        previousVelocity = currentVelocity;

        // 2. GYROSCOPE: Angular velocity
        angularVelocity = transform.InverseTransformDirection(rb.angularVelocity);

        if (addNoise)
        {
            angularVelocity += new Vector3(
                Random.Range(-gyroNoise, gyroNoise),
                Random.Range(-gyroNoise, gyroNoise),
                Random.Range(-gyroNoise, gyroNoise)
            );
        }

        // 3. ORIENTATION: Current rotation
        orientation = transform.eulerAngles;

        // 4. MAGNETOMETER: Pointing north (simplified)
        // In real world, affected by local magnetic fields
        Vector3 northDirection = Vector3.forward; // World north
        magneticField = transform.InverseTransformDirection(northDirection);
    }

    // Public API for other scripts
    public Vector3 GetAcceleration() { return acceleration; }
    public Vector3 GetGyroscope() { return angularVelocity; }
    public Vector3 GetOrientation() { return orientation; }
    public Quaternion GetOrientationQuaternion() { return transform.rotation; }

    void OnGUI()
    {
        // Display IMU data (for debugging)
        GUILayout.BeginArea(new Rect(10, 10, 300, 200));
        GUILayout.Label("IMU Data:");
        GUILayout.Label($"Accel (m/s²): {acceleration:F3}");
        GUILayout.Label($"Gyro (rad/s): {angularVelocity:F3}");
        GUILayout.Label($"Orientation (°): {orientation:F1}");
        GUILayout.Label($"Magnetic: {magneticField:F3}");
        GUILayout.EndArea();
    }
}
```

**Usage**:
- Attach to robot's main body (base_link or torso)
- Requires Rigidbody component (physics simulation)
- Runs in `FixedUpdate()` for stable physics rate

**Visualization in UI**:
- On-screen display shows real-time IMU readings
- Acceleration changes when robot moves
- Gyroscope active during rotations
- Orientation shows current tilt

**Use Case: Balance Controller**
- Read IMU orientation: `imu.GetOrientation()`
- Detect tilt: `if (Mathf.Abs(orientation.x) > 10f) → "Tilting!"`
- Apply corrective torque to joints
- Keep humanoid upright during walking

---

## Sensor Fusion: Combining Data

**Real robots don't use sensors in isolation.**

**Example: Robot Navigation**

1. **LiDAR**: "Wall detected 2m ahead"
2. **Camera**: "Wall is red brick"
3. **IMU**: "I'm moving forward at 0.5 m/s, tilted 2° left"
4. **Odometry** (from wheels/joints): "Traveled 3.2m in last 5 seconds"

**Fusion Algorithm** (Extended Kalman Filter, Particle Filter):
- Combines all sensor data
- Reduces noise, improves accuracy
- Outputs: Precise position, velocity, orientation

**Unity Advantage**: Test fusion algorithms with perfect ground truth
- Unity knows exact robot position (no sensor error)
- Compare fusion output vs. truth → measure accuracy
- Tune algorithm until confident
- Deploy to real robot

---

## Sensor Placement Matters

**Where you mount sensors affects performance.**

### Camera Placement

**Head-mounted** (human eye height):
- ✅ Natural perspective for human environments
- ✅ Good for face recognition, navigation
- ❌ Can't see robot's own feet
- ❌ Blocked by arms during manipulation

**Chest-mounted** (lower):
- ✅ Sees manipulation workspace (hands, objects)
- ✅ Less occlusion from arms
- ❌ Lower viewpoint (can't see over obstacles)

**Multiple cameras** (best):
- Front camera: Navigation, object detection
- Hand cameras (2): Manipulation close-up
- Rear camera: Backward safety

### LiDAR Placement

**Top of head** (highest point):
- ✅ 360° unobstructed view
- ✅ Sees over low obstacles
- ❌ Can't detect low objects (below sensor plane)

**Chest level**:
- ✅ Detects waist-height obstacles (tables, counters)
- ❌ May be blocked by arms

**Multi-layer LiDAR** (e.g., Velodyne VLP-16):
- 16 vertical layers → captures high and low obstacles
- Recommended for humanoids

### IMU Placement

**Center of mass** (torso):
- ✅ Measures robot's true motion
- ✅ Best for balance algorithms

**Head**:
- ❌ Exaggerates movements (long moment arm)
- ✅ Good for camera stabilization (detect head shake)

---

## Practical Example: Office Navigation Robot

**Scenario**: Humanoid delivers documents in office

**Sensors Needed**:

1. **Front RGB Camera** (head-mounted)
   - Detect people, read room numbers, find target desk
   - Resolution: 1920×1080
   - FOV: 90° (wide for navigation)

2. **Depth Camera** (chest-mounted)
   - Avoid obstacles (chairs, boxes)
   - 3D mapping of environment
   - Range: 0.5m-5m

3. **LiDAR** (head-mounted)
   - SLAM for map building
   - Localization in mapped environment
   - 16-layer, 360°, 100m range

4. **IMU** (torso)
   - Detect if tilting (balance control)
   - Estimate traveled distance (odometry)

**Unity Setup**:
- Create office environment (desks, chairs, walls)
- Attach all 4 sensors to robot
- Scripts:
  - `RobotCameraCapture.cs` → RGB images
  - `DepthCamera.cs` → Depth maps
  - `LiDARSensor.cs` → Point clouds
  - `IMUSensor.cs` → Acceleration, gyroscope

**Test Scenario**:
- Robot spawns at entrance
- Task: Navigate to desk #42
- Obstacles: Moving people (animated avatars), scattered chairs
- Sensors:
  - LiDAR: Builds map, localizes position
  - Depth camera: Avoids close obstacles
  - RGB camera: Reads desk numbers
  - IMU: Confirms straight-line travel, detects bumps

**Output**:
- 1,000 RGB images (for desk number recognition training)
- 50 LiDAR point clouds (for SLAM testing)
- 100 depth maps (for obstacle avoidance validation)
- Continuous IMU data (for odometry comparison)

---

## Key Concepts Summary

**Sensors Are Essential**:
- Robots navigate using sensors (like humans use eyes, ears, balance)
- Without sensors: blind, deaf, disoriented

**Three Core Sensors**:
- **Cameras**: See colors, textures, objects (object detection, navigation)
- **LiDAR**: Measure distances in 360° (SLAM, mapping, localization)
- **IMU**: Detect orientation, acceleration (balance, odometry)

**Unity Simulation Benefits**:
- Generate synthetic data (1,000s of images overnight)
- Test extreme conditions (darkness, fog, rain)
- Perfect ground truth for algorithm validation
- No hardware risk (crash 1,000 times, $0 cost)

**Sensor Fusion**:
- Combine multiple sensors for better accuracy
- Unity provides perfect ground truth for validation

**Placement Matters**:
- Camera height affects what robot sees
- LiDAR position determines coverage
- IMU at center-of-mass for accurate motion

---

## What's Next

Now that your robot has "senses," you'll learn how to connect Unity to ROS 2—enabling real robotics integration.

**Next Topics**:
- **Page 16**: Unity Robotics Hub (ROS 2 Integration)
- **Page 17**: Connecting Unity to Gazebo (Dual Simulation)
- **Page 18**: Generating Synthetic Training Data at Scale
- **Page 19**: Domain Randomization for Robust Perception

**The Journey**:
- ✅ Understood Unity's role (graphics, HRI, training data)
- ✅ Built your first scene (environment, lighting, physics)
- ✅ **Simulated sensors** (cameras, LiDAR, IMU)
- 🔜 Connect to ROS 2 for real robotics workflows
- 🔜 Generate massive synthetic datasets
- 🔜 Train vision models without real photos

**You're Building Toward**: A complete perception pipeline where your robot "sees" the world through simulated sensors, generates training data in Unity, and deploys vision algorithms to real hardware—all without a single real photo.

---

*"A robot without sensors is a body without senses. Unity gives your robot eyes, ears, and balance—at zero cost."*
