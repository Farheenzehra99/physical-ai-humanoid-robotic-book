# Unity_Vis Skill

## Purpose
Provides Unity-based 3D visualization, human-robot interaction (HRI) environments, and high-quality rendering for robotics applications. This skill enables photorealistic visualization, VR/AR integration, and interactive environments beyond physics simulation requirements.

## Core Capabilities
- High-quality 3D rendering and visualization
- Unity-ROS2 integration
- Human-robot interaction scene creation
- VR/AR experience development
- Real-time animation and articulation
- Custom UI/UX for robot control
- Video capture and recording
- Interactive object manipulation
- Lighting and post-processing effects

## Pipeline

### 1. Project Setup
```csharp
// Unity-ROS2 connection
using ROS2;

public class RobotVisualizer : MonoBehaviour
{
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        ros2Node = ros2Unity.CreateNode("unity_visualizer");
    }
}
```

### 2. Robot Model Import
- Import URDF via Unity Robotics Hub
- Configure articulation body components
- Set up materials and shaders
- Create animation controllers

### 3. Scene Creation
- Environment design (indoor/outdoor)
- Lighting setup (directional, point, area)
- Camera configuration (perspective, orthographic)
- Interactive elements placement

### 4. ROS2 Integration
- Subscribe to joint states
- Visualize sensor data (point clouds, images)
- Publish user interactions
- Transform synchronization

## Key Functions

### `ImportURDF(urdfPath, scale)`
Imports URDF model into Unity scene with proper articulation.

### `SubscribeJointStates(topic, callback)`
Subscribes to ROS2 joint state messages and updates robot visualization.

### `CreateEnvironment(type, objects)`
Generates environment scene (home, office, warehouse, outdoor).

### `SetupCamera(mode, target, settings)`
Configures camera for visualization or data capture.

### `EnableVRMode(headset)`
Initializes VR rendering for immersive interaction.

## Examples

### Example 1: Joint State Subscriber
```csharp
using ROS2;
using sensor_msgs.msg;

public class JointVisualizer : MonoBehaviour
{
    private ISubscription<JointState> jointSub;
    private ArticulationBody[] joints;

    void Start()
    {
        var node = GetComponent<ROS2UnityComponent>().CreateNode("joint_vis");
        jointSub = node.CreateSubscription<JointState>(
            "/joint_states",
            msg => UpdateJointPositions(msg)
        );
        joints = GetComponentsInChildren<ArticulationBody>();
    }

    void UpdateJointPositions(JointState msg)
    {
        for (int i = 0; i < msg.Position.Length && i < joints.Length; i++)
        {
            var drive = joints[i].xDrive;
            drive.target = (float)msg.Position[i] * Mathf.Rad2Deg;
            joints[i].xDrive = drive;
        }
    }
}
```

### Example 2: Interactive Object Placement
```csharp
public class InteractiveObjectSpawner : MonoBehaviour
{
    public GameObject objectPrefab;

    void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out RaycastHit hit))
            {
                Instantiate(objectPrefab, hit.point, Quaternion.identity);
            }
        }
    }
}
```

### Example 3: Camera Follow Robot
```csharp
public class RobotCamera : MonoBehaviour
{
    public Transform robotTransform;
    public Vector3 offset = new Vector3(0, 2, -3);
    public float smoothSpeed = 0.125f;

    void LateUpdate()
    {
        Vector3 desiredPosition = robotTransform.position + offset;
        Vector3 smoothedPosition = Vector3.Lerp(
            transform.position,
            desiredPosition,
            smoothSpeed
        );
        transform.position = smoothedPosition;
        transform.LookAt(robotTransform);
    }
}
```

### Example 4: HRI Scene Setup
```csharp
public class HRIEnvironment : MonoBehaviour
{
    void CreateLivingRoom()
    {
        // Floor
        CreatePlane("Floor", new Vector3(0, 0, 0), new Vector3(10, 1, 10));

        // Furniture
        CreateCube("Table", new Vector3(0, 0.5f, 2), new Vector3(1.5f, 1, 0.8f));
        CreateCube("Chair", new Vector3(0, 0.5f, 3), new Vector3(0.5f, 1, 0.5f));

        // Lighting
        CreateLight("MainLight", LightType.Directional, new Vector3(50, -30, 0));
        CreateLight("AmbientLight", LightType.Point, new Vector3(0, 3, 0));
    }
}
```

## Dependencies
- Unity 2021.3 LTS or newer
- Unity Robotics Hub package
- ROS-TCP-Connector
- Unity URDF Importer
- TextMeshPro
- Universal Render Pipeline (optional, for better graphics)
- VR/AR packages (if using XR features)

## Best Practices
- Use Universal Render Pipeline for better performance
- Optimize mesh complexity for real-time rendering
- Use LOD (Level of Detail) for distant objects
- Implement object pooling for spawned objects
- Use coroutines for smooth animations
- Separate visualization from physics simulation
- Cache component references in Start()
- Use layers for selective rendering
- Implement proper resource cleanup
- Test performance with profiler

## Common Scene Types

### Indoor Navigation
- Residential (living room, kitchen, bedroom)
- Office (cubicles, meeting rooms)
- Hospital (corridors, patient rooms)
- Warehouse (shelves, pallets, forklifts)

### Outdoor Environments
- Urban (streets, sidewalks, buildings)
- Park (trees, benches, paths)
- Construction site
- Natural terrain

### HRI Scenarios
- Handover tasks
- Following and guiding
- Collaborative manipulation
- Social interaction spaces

### Data Collection
- Synthetic dataset generation
- Multi-view captures
- Segmentation masks
- Depth maps

## Unity-ROS2 Message Types

### Standard Subscriptions
- `sensor_msgs/JointState` - Joint positions/velocities
- `geometry_msgs/TransformStamped` - Object poses
- `sensor_msgs/PointCloud2` - LiDAR visualization
- `sensor_msgs/Image` - Camera feed display

### Standard Publications
- `geometry_msgs/Pose` - User-placed objects
- `std_msgs/Bool` - UI button clicks
- `std_msgs/String` - User commands

## Performance Optimization

### Real-Time Visualization
- Target 60 FPS for smooth experience
- Use GPU instancing for repeated objects
- Reduce draw calls with batching
- Optimize shader complexity
- Use occlusion culling

### VR/AR Mode
- Maintain 90 FPS minimum
- Reduce polygon count
- Simplify shaders
- Optimize texture sizes
- Use single-pass stereo rendering

## Integration Points
- Receives joint states from ROS2_Core
- Visualizes URDF models from URDF_Designer
- Displays simulation alongside Gazebo_Sim
- Renders environments for VLA_Controller vision
- Provides UI for Hardware_Proxy control
- Creates datasets for IsaacSim_Pipeline training
