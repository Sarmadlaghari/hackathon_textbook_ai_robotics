---
title: High-Fidelity Rendering in Unity
sidebar_label: "Week 5: High-Fidelity Rendering in Unity"
sidebar_position: 2
---

# High-Fidelity Rendering in Unity

## Learning Objectives

By the end of this week, students will be able to:
- Understand the fundamentals of high-fidelity rendering in Unity
- Configure Unity for photorealistic visualization
- Implement advanced lighting and material systems
- Integrate Unity with robotics simulation frameworks
- Create realistic sensor simulation in Unity environments
- Optimize rendering performance for real-time applications

## Introduction

High-fidelity rendering in Unity has become increasingly important for robotics applications, particularly for creating realistic training environments for AI systems and simulating sensor data with photorealistic quality. Unity's advanced rendering pipeline, including the High Definition Render Pipeline (HDRP) and Universal Render Pipeline (URP), enables the creation of photorealistic environments that can be used for synthetic data generation, sensor simulation, and human-in-the-loop testing. This week explores the fundamentals of high-fidelity rendering in Unity and its applications in robotics simulation.

Unity's rendering capabilities have evolved significantly, offering features like real-time ray tracing, physically-based rendering (PBR), global illumination, and advanced post-processing effects. These features make Unity an excellent platform for creating realistic digital twins of physical environments, which are essential for training robust perception systems in robotics. The integration of Unity with robotics frameworks enables seamless transfer of simulated data to real-world applications.

## Theory

### Unity Rendering Pipelines

Unity offers three main rendering pipelines optimized for different use cases:

#### Built-in Render Pipeline
The legacy rendering pipeline that offers basic rendering capabilities and is suitable for simple applications. While still functional, it lacks many of the advanced features available in the newer pipelines.

#### Universal Render Pipeline (URP)
A lightweight, flexible rendering pipeline designed for performance across a wide range of platforms. URP provides a good balance between visual quality and performance, making it suitable for mobile robotics applications and applications requiring high frame rates.

Key features of URP:
- Lightweight and efficient
- Supports 2D and 3D rendering
- Customizable render passes
- Built-in post-processing effects
- Shader Graph integration

#### High Definition Render Pipeline (HDRP)
A state-of-the-art rendering pipeline designed for high-fidelity visuals on powerful hardware. HDRP is ideal for applications requiring photorealistic rendering, such as digital twin creation and high-quality sensor simulation.

Key features of HDRP:
- Physically-based rendering
- Real-time ray tracing
- Advanced lighting models
- Global illumination (Baked and realtime)
- Volumetric effects
- Advanced post-processing stack

### Physically-Based Rendering (PBR)

Physically-Based Rendering is a methodology that simulates light-object interactions using real-world physics principles. PBR materials in Unity are defined by several key properties:

#### Albedo (Base Color)
The base color of the material without lighting considerations. This represents the color of the surface when illuminated by white light.

#### Metallic
Controls whether a surface behaves like a metal or a non-metal. Metallic surfaces reflect light as colored reflections, while non-metallic surfaces reflect light as white highlights.

#### Smoothness/Roughness
Controls the microsurface detail of the material, affecting how light scatters. Smooth surfaces create sharp reflections, while rough surfaces create diffuse reflections.

#### Normal Maps
Provide detailed surface geometry information without increasing polygon count. Normal maps create the illusion of complex surface details like scratches, bumps, and grooves.

#### Occlusion Maps
Simulate the shadowing effect of small surface details, enhancing the perception of depth and contact between surfaces.

### Lighting Systems in Unity

Unity provides several lighting systems that can be used to create realistic illumination:

#### Real-time Lighting
Lights that affect objects dynamically during gameplay. Real-time lighting provides interactive illumination but can be computationally expensive.

Types of real-time lights:
- Directional lights: Simulate distant light sources like the sun
- Point lights: Emit light in all directions from a point
- Spot lights: Emit light in a cone shape
- Area lights: Emit light from a surface area (baked only in some pipelines)

#### Baked Lighting
Lighting that is precomputed and stored in lightmaps. Baked lighting provides high-quality global illumination but cannot change at runtime.

#### Mixed Lighting
Combines real-time and baked lighting, allowing for static lighting to be baked while enabling dynamic shadows from moving objects.

### Unity Sensor Simulation

Unity can simulate various sensor types used in robotics, providing realistic sensor data for testing and training:

#### Camera Simulation
Unity's camera components can simulate various camera types:
- RGB cameras for visual perception
- Depth cameras for 3D reconstruction
- Semantic segmentation cameras for object recognition
- Stereo cameras for depth perception

#### LIDAR Simulation
LIDAR sensors can be simulated using raycasting techniques, providing accurate distance measurements and point cloud data similar to real LIDAR sensors.

#### IMU Simulation
Inertial measurement units can be simulated by tracking the acceleration and rotation of objects in the Unity scene, providing realistic IMU data for navigation algorithms.

### Unity Robotics Integration

Unity provides several tools and packages for robotics integration:

#### Unity Robotics Hub
A collection of tools and packages that facilitate robotics development in Unity, including:
- ROS# for ROS communication
- ML-Agents for reinforcement learning
- Unity Perception for synthetic data generation

#### Unity Simulation Framework
The framework enables the creation of complex simulation environments with realistic physics and rendering, allowing for comprehensive testing of robotics algorithms.

#### Asset Packages for Robotics
Unity provides specialized asset packages for robotics simulation, including:
- Procedural environment generation
- Physics-based robot models
- Sensor simulation tools

### Performance Optimization for Real-Time Rendering

High-fidelity rendering can be computationally intensive. Several optimization strategies help maintain performance:

#### Level of Detail (LOD)
Implementing multiple levels of detail for 3D models allows the renderer to use simpler representations when objects are far from the camera.

#### Occlusion Culling
Hiding objects that are not visible to the camera based on occlusion data reduces unnecessary rendering calculations.

#### Dynamic Batching
Combining multiple small objects with the same materials into single draw calls reduces the number of rendering operations.

#### Shader Optimization
Using efficient shaders and minimizing overdraw helps maintain high frame rates in complex scenes.

## Code Examples

### Unity C# Script for Camera Sensor Simulation

Here's an example of how to implement camera sensor simulation in Unity:

```csharp
using System;
using UnityEngine;
using System.Collections;
using System.IO;

[RequireComponent(typeof(Camera))]
public class CameraSensor : MonoBehaviour
{
    [Header("Camera Settings")]
    public int imageWidth = 640;
    public int imageHeight = 480;
    public float fieldOfView = 60f;
    public string outputDirectory = "CameraOutput";

    [Header("Sensor Simulation")]
    public bool simulateDepth = false;
    public float minDepth = 0.1f;
    public float maxDepth = 100f;
    public bool simulateSemanticSegmentation = false;

    private Camera cam;
    private RenderTexture renderTexture;
    private Texture2D outputTexture;

    void Start()
    {
        cam = GetComponent<Camera>();
        SetupCamera();
        CreateRenderTexture();
    }

    void SetupCamera()
    {
        cam.fieldOfView = fieldOfView;
        cam.enabled = false; // Disable default rendering
    }

    void CreateRenderTexture()
    {
        renderTexture = new RenderTexture(imageWidth, imageHeight, 24);
        renderTexture.format = RenderTextureFormat.ARGB32;
        renderTexture.antiAliasing = 1;
        renderTexture.Create();

        outputTexture = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
    }

    public void CaptureImage()
    {
        // Set the camera's target texture
        cam.targetTexture = renderTexture;

        // Render the camera
        cam.Render();

        // Read the render texture into the output texture
        RenderTexture.active = renderTexture;
        outputTexture.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        outputTexture.Apply();

        // Reset render texture
        RenderTexture.active = null;
        cam.targetTexture = null;

        // Save the image
        SaveImage(outputTexture, "rgb_image");
    }

    public void CaptureDepthImage()
    {
        if (!simulateDepth) return;

        // Create a temporary render texture for depth
        RenderTexture depthRT = RenderTexture.GetTemporary(
            imageWidth, imageHeight, 24, RenderTextureFormat.RFloat);

        // Set up camera for depth rendering
        cam.SetTargetBuffers(depthRT.colorBuffer, depthRT.depthBuffer);
        cam.RenderWithShader(Shader.Find("Hidden/DepthOnly"), "");

        // Read depth data
        RenderTexture.active = depthRT;
        Texture2D depthTexture = new Texture2D(imageWidth, imageHeight, TextureFormat.RFloat, false);
        depthTexture.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        depthTexture.Apply();

        // Reset
        RenderTexture.active = null;
        cam.ResetReplacementShader();

        // Save the depth image
        SaveImage(depthTexture, "depth_image");

        // Clean up
        RenderTexture.ReleaseTemporary(depthRT);
        DestroyImmediate(depthTexture);
    }

    private void SaveImage(Texture2D texture, string prefix)
    {
        byte[] bytes = texture.EncodeToPNG();

        // Create directory if it doesn't exist
        string fullDir = Path.Combine(Application.dataPath, outputDirectory);
        if (!Directory.Exists(fullDir))
        {
            Directory.CreateDirectory(fullDir);
        }

        // Generate filename with timestamp
        string filename = $"{prefix}_{DateTime.Now:yyyyMMdd_HHmmss}.png";
        string filepath = Path.Combine(fullDir, filename);

        File.WriteAllBytes(filepath, bytes);
        Debug.Log($"Image saved to: {filepath}");
    }

    void OnDestroy()
    {
        if (renderTexture != null)
        {
            renderTexture.Release();
        }
        if (outputTexture != null)
        {
            DestroyImmediate(outputTexture);
        }
    }
}
```

### Unity C# Script for LIDAR Simulation

Here's an example of how to simulate LIDAR sensors in Unity:

```csharp
using System.Collections.Generic;
using UnityEngine;

public class LIDARSensor : MonoBehaviour
{
    [Header("LIDAR Configuration")]
    public int numberOfRays = 360;
    public float minAngle = -90f;
    public float maxAngle = 90f;
    public float maxDistance = 20f;
    public LayerMask detectionMask = -1;
    public string outputTopic = "/laser_scan";

    [Header("Performance")]
    public float updateRate = 10f; // Hz
    public bool visualizeRays = true;

    private LineRenderer lineRenderer;
    private float updateInterval;
    private float lastUpdateTime;

    // Data structure for LIDAR output
    [System.Serializable]
    public class LaserScanData
    {
        public float[] ranges;
        public float angle_min;
        public float angle_max;
        public float angle_increment;
        public float time_increment;
        public float scan_time;
        public float range_min;
        public float range_max;
    }

    void Start()
    {
        updateInterval = 1f / updateRate;
        lastUpdateTime = 0f;

        if (visualizeRays)
        {
            SetupLineRenderer();
        }
    }

    void SetupLineRenderer()
    {
        lineRenderer = gameObject.AddComponent<LineRenderer>();
        lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
        lineRenderer.widthMultiplier = 0.02f;
        lineRenderer.positionCount = numberOfRays + 1;
        lineRenderer.useWorldSpace = false;
    }

    void Update()
    {
        if (Time.time - lastUpdateTime >= updateInterval)
        {
            PerformLIDARScan();
            lastUpdateTime = Time.time;
        }
    }

    public LaserScanData PerformLIDARScan()
    {
        LaserScanData scanData = new LaserScanData();
        scanData.ranges = new float[numberOfRays];
        scanData.angle_min = minAngle * Mathf.Deg2Rad;
        scanData.angle_max = maxAngle * Mathf.Deg2Rad;
        scanData.angle_increment = (maxAngle - minAngle) * Mathf.Deg2Rad / numberOfRays;
        scanData.time_increment = 0f;
        scanData.scan_time = updateInterval;
        scanData.range_min = 0.1f;
        scanData.range_max = maxDistance;

        float angleStep = (maxAngle - minAngle) / numberOfRays;

        for (int i = 0; i < numberOfRays; i++)
        {
            float angle = minAngle + i * angleStep;
            float radians = angle * Mathf.Deg2Rad;

            // Calculate ray direction in local space
            Vector3 rayDirection = new Vector3(
                Mathf.Cos(radians),
                0f,
                Mathf.Sin(radians)
            );

            // Transform to world space
            Vector3 worldRayDirection = transform.TransformDirection(rayDirection);

            // Perform raycast
            RaycastHit hit;
            if (Physics.Raycast(transform.position, worldRayDirection, out hit, maxDistance, detectionMask))
            {
                scanData.ranges[i] = hit.distance;

                // Visualize ray if enabled
                if (visualizeRays && lineRenderer != null)
                {
                    lineRenderer.SetPosition(i, Vector3.zero);
                    lineRenderer.SetPosition(i + 1, transform.InverseTransformPoint(hit.point));
                }
            }
            else
            {
                scanData.ranges[i] = float.PositiveInfinity;

                // Visualize ray to max distance if enabled
                if (visualizeRays && lineRenderer != null)
                {
                    Vector3 maxPoint = transform.InverseTransformPoint(
                        transform.position + worldRayDirection * maxDistance
                    );
                    lineRenderer.SetPosition(i, Vector3.zero);
                    lineRenderer.SetPosition(i + 1, maxPoint);
                }
            }
        }

        return scanData;
    }

    // Method to get point cloud from LIDAR data
    public List<Vector3> GetPointCloud(LaserScanData scanData)
    {
        List<Vector3> pointCloud = new List<Vector3>();
        Vector3 sensorPosition = transform.position;

        for (int i = 0; i < scanData.ranges.Length; i++)
        {
            float range = scanData.ranges[i];
            if (range > scanData.range_min && range < scanData.range_max)
            {
                float angle = scanData.angle_min + i * scanData.angle_increment;

                // Calculate point in 2D plane (for 2D LIDAR)
                Vector3 point = new Vector3(
                    range * Mathf.Cos(angle),
                    0f,
                    range * Mathf.Sin(angle)
                );

                // Transform to world coordinates
                point = transform.TransformPoint(point);
                pointCloud.Add(point);
            }
        }

        return pointCloud;
    }

    // Method to simulate 3D LIDAR by stacking 2D scans
    public List<Vector3> Get3DPointCloud(int verticalBeams = 16, float verticalFOV = 30f)
    {
        List<Vector3> pointCloud = new List<Vector3>();

        for (int v = 0; v < verticalBeams; v++)
        {
            float verticalAngle = -verticalFOV / 2 + (v * verticalFOV / (verticalBeams - 1));

            // Rotate the sensor temporarily for vertical angle
            Vector3 originalEuler = transform.eulerAngles;
            transform.eulerAngles = new Vector3(
                originalEuler.x + verticalAngle,
                originalEuler.y,
                originalEuler.z
            );

            // Perform scan at this vertical angle
            LaserScanData scanData = PerformLIDARScan();
            List<Vector3> scanPoints = GetPointCloud(scanData);
            pointCloud.AddRange(scanPoints);

            // Restore original rotation
            transform.eulerAngles = originalEuler;
        }

        return pointCloud;
    }

    void OnValidate()
    {
        numberOfRays = Mathf.Clamp(numberOfRays, 1, 10000);
        maxDistance = Mathf.Clamp(maxDistance, 0.1f, 1000f);
        updateRate = Mathf.Clamp(updateRate, 0.1f, 100f);
    }
}
```

### Unity C# Script for IMU Simulation

Here's an example of how to simulate IMU sensors in Unity:

```csharp
using System.Collections;
using UnityEngine;

public class IMUSensor : MonoBehaviour
{
    [Header("IMU Configuration")]
    public float updateRate = 100f; // Hz
    public bool includeGyroscope = true;
    public bool includeAccelerometer = true;
    public bool includeMagnetometer = false;

    [Header("Noise Parameters")]
    public float accelerometerNoise = 0.01f;
    public float gyroscopeNoise = 0.01f;
    public float magnetometerNoise = 0.1f;

    [Header("Gravity")]
    public Vector3 gravity = new Vector3(0, -9.81f, 0);

    private float updateInterval;
    private float lastUpdateTime;

    // Data structure for IMU output
    [System.Serializable]
    public class IMUData
    {
        public Vector3 linear_acceleration;
        public Vector3 angular_velocity;
        public Vector3 magnetic_field;
        public Quaternion orientation;
    }

    // Store previous state for velocity calculation
    private Vector3 previousPosition;
    private Quaternion previousRotation;
    private float previousTime;

    void Start()
    {
        updateInterval = 1f / updateRate;
        lastUpdateTime = Time.time;

        previousPosition = transform.position;
        previousRotation = transform.rotation;
        previousTime = Time.time;
    }

    void Update()
    {
        if (Time.time - lastUpdateTime >= updateInterval)
        {
            PublishIMUData();
            lastUpdateTime = Time.time;
        }
    }

    public IMUData GetIMUData()
    {
        IMUData imuData = new IMUData();

        // Calculate linear acceleration
        if (includeAccelerometer)
        {
            Vector3 currentPosition = transform.position;
            float currentTime = Time.time;

            // Calculate velocity from position changes
            Vector3 velocity = (currentPosition - previousPosition) / (currentTime - previousTime);
            Vector3 previousVelocity = (previousPosition - GetPreviousPosition(2)) / (currentTime - previousTime);

            // Calculate acceleration from velocity changes
            Vector3 acceleration = (velocity - previousVelocity) / (currentTime - previousTime);

            // Add gravity compensation (remove gravity from linear acceleration)
            imuData.linear_acceleration = acceleration - transform.InverseTransformDirection(gravity);

            // Add noise
            imuData.linear_acceleration += AddNoiseVector(accelerometerNoise);
        }

        // Calculate angular velocity
        if (includeGyroscope)
        {
            Quaternion currentRotation = transform.rotation;
            float currentTime = Time.time;

            // Calculate angular velocity from rotation changes
            Quaternion deltaRotation = currentRotation * Quaternion.Inverse(previousRotation);
            Vector3 angularVelocity = new Vector3();

            float deltaTime = currentTime - previousTime;
            if (deltaTime > 0)
            {
                // Convert quaternion to angular velocity
                float angle;
                Vector3 axis;
                deltaRotation.ToAngleAxis(out angle, out axis);

                // Convert from degrees to radians and normalize by time
                angularVelocity = axis * Mathf.Deg2Rad * angle / deltaTime;
            }

            imuData.angular_velocity = transform.InverseTransformDirection(angularVelocity);

            // Add noise
            imuData.angular_velocity += AddNoiseVector(gyroscopeNoise);
        }

        // Simulate magnetic field
        if (includeMagnetometer)
        {
            // In a real implementation, this would be based on geographic location
            // For simulation, we'll use a fixed magnetic field vector
            imuData.magnetic_field = transform.InverseTransformDirection(new Vector3(22.9f, 0f, 44.4f)); // Approximate magnetic field in microteslas

            // Add noise
            imuData.magnetic_field += AddNoiseVector(magnetometerNoise);
        }

        // Calculate orientation
        imuData.orientation = transform.rotation;

        // Update previous state
        previousPosition = transform.position;
        previousRotation = transform.rotation;
        previousTime = Time.time;

        return imuData;
    }

    private Vector3 GetPreviousPosition(int framesAgo)
    {
        // In a real implementation, you'd store multiple previous positions
        // For simplicity, we'll just return the last known position
        return previousPosition;
    }

    private Vector3 AddNoiseVector(float noiseLevel)
    {
        return new Vector3(
            Random.Range(-noiseLevel, noiseLevel),
            Random.Range(-noiseLevel, noiseLevel),
            Random.Range(-noiseLevel, noiseLevel)
        );
    }

    public void PublishIMUData()
    {
        IMUData data = GetIMUData();

        // In a real implementation, this would publish to ROS or another messaging system
        Debug.Log($"IMU Data - Accel: {data.linear_acceleration}, Gyro: {data.angular_velocity}");
    }

    // Method to get acceleration in world frame
    public Vector3 GetWorldAcceleration()
    {
        IMUData data = GetIMUData();
        return transform.TransformDirection(data.linear_acceleration);
    }

    // Method to get angular velocity in world frame
    public Vector3 GetWorldAngularVelocity()
    {
        IMUData data = GetIMUData();
        return transform.TransformDirection(data.angular_velocity);
    }
}
```

### Unity C# Script for Physics-Based Environment Generation

Here's an example of how to procedurally generate physics-based environments in Unity:

```csharp
using System.Collections.Generic;
using UnityEngine;

public class PhysicsEnvironmentGenerator : MonoBehaviour
{
    [Header("Terrain Generation")]
    public int terrainWidth = 200;
    public int terrainLength = 200;
    public float terrainHeight = 20f;
    public int resolution = 256;

    [Header("Object Placement")]
    public GameObject[] obstaclePrefabs;
    public int minObstacles = 10;
    public int maxObstacles = 50;
    public float placementAreaPadding = 5f;

    [Header("Material Properties")]
    public PhysicMaterial defaultMaterial;
    public float dynamicFriction = 0.6f;
    public float staticFriction = 0.6f;
    public float bounciness = 0.1f;

    [Header("Environment Features")]
    public bool generateRandomTerrain = true;
    public bool addStaticObstacles = true;
    public bool addDynamicObstacles = true;
    public bool addRamps = false;
    public int numRamps = 3;

    private List<GameObject> spawnedObjects = new List<GameObject>();
    private float[,] heightMap;

    void Start()
    {
        GenerateEnvironment();
    }

    public void GenerateEnvironment()
    {
        ClearEnvironment();

        if (generateRandomTerrain)
        {
            GenerateTerrain();
        }

        if (addStaticObstacles)
        {
            PlaceStaticObstacles();
        }

        if (addDynamicObstacles)
        {
            PlaceDynamicObstacles();
        }

        if (addRamps)
        {
            PlaceRamps();
        }
    }

    void ClearEnvironment()
    {
        // Destroy previously spawned objects
        foreach (GameObject obj in spawnedObjects)
        {
            if (obj != null)
            {
                DestroyImmediate(obj);
            }
        }
        spawnedObjects.Clear();
    }

    void GenerateTerrain()
    {
        // Create terrain programmatically
        GameObject terrainObj = new GameObject("GeneratedTerrain");
        Terrain terrain = terrainObj.AddComponent<Terrain>();
        TerrainCollider terrainCollider = terrainObj.AddComponent<TerrainCollider>();

        // Create terrain data
        TerrainData terrainData = new TerrainData();
        terrainData.heightmapResolution = resolution;
        terrainData.size = new Vector3(terrainWidth, terrainHeight, terrainLength);

        // Generate heightmap
        heightMap = GenerateHeightMap(resolution, resolution);
        terrainData.SetHeights(0, 0, heightMap);

        // Assign the terrain data
        terrain.terrainData = terrainData;
        terrainCollider.terrainData = terrainData;

        spawnedObjects.Add(terrainObj);
    }

    float[,] GenerateHeightMap(int width, int height)
    {
        float[,] heights = new float[width, height];

        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < height; y++)
            {
                // Generate Perlin noise-based terrain
                float xCoord = (float)x / width * 10f;
                float yCoord = (float)y / height * 10f;

                // Multiple octaves for more natural terrain
                float elevation = 0f;
                float amplitude = 1f;
                float frequency = 1f;
                float persistence = 0.5f;

                for (int octave = 0; octave < 4; octave++)
                {
                    elevation += Mathf.PerlinNoise(xCoord * frequency, yCoord * frequency) * amplitude;
                    amplitude *= persistence;
                    frequency *= 2f;
                }

                // Normalize to 0-1 range and apply to height
                heights[x, y] = elevation / 4f; // Divide by number of octaves for normalization
            }
        }

        return heights;
    }

    void PlaceStaticObstacles()
    {
        int numObstacles = Random.Range(minObstacles, maxObstacles + 1);

        for (int i = 0; i < numObstacles; i++)
        {
            if (obstaclePrefabs.Length == 0) continue;

            // Select random obstacle prefab
            GameObject obstaclePrefab = obstaclePrefabs[Random.Range(0, obstaclePrefabs.Length)];

            // Generate random position within bounds
            Vector3 position = new Vector3(
                Random.Range(placementAreaPadding, terrainWidth - placementAreaPadding),
                10f, // Start above ground to let it fall
                Random.Range(placementAreaPadding, terrainLength - placementAreaPadding)
            );

            // Create obstacle
            GameObject obstacle = Instantiate(obstaclePrefab, position, Quaternion.identity);

            // Add rigidbody if it doesn't have one
            if (obstacle.GetComponent<Rigidbody>() == null)
            {
                Rigidbody rb = obstacle.AddComponent<Rigidbody>();
                rb.isKinematic = true; // Static obstacles
            }

            // Set material properties
            SetPhysicsMaterial(obstacle);

            spawnedObjects.Add(obstacle);
        }
    }

    void PlaceDynamicObstacles()
    {
        int numObstacles = Random.Range(minObstacles / 2, maxObstacles / 2 + 1);

        for (int i = 0; i < numObstacles; i++)
        {
            if (obstaclePrefabs.Length == 0) continue;

            // Select random obstacle prefab
            GameObject obstaclePrefab = obstaclePrefabs[Random.Range(0, obstaclePrefabs.Length)];

            // Generate random position within bounds
            Vector3 position = new Vector3(
                Random.Range(placementAreaPadding, terrainWidth - placementAreaPadding),
                10f, // Start above ground
                Random.Range(placementAreaPadding, terrainLength - placementAreaPadding)
            );

            // Create obstacle
            GameObject obstacle = Instantiate(obstaclePrefab, position, Quaternion.identity);

            // Add rigidbody for physics simulation
            Rigidbody rb = obstacle.GetComponent<Rigidbody>();
            if (rb == null)
            {
                rb = obstacle.AddComponent<Rigidbody>();
            }

            // Set as dynamic
            rb.isKinematic = false;
            rb.useGravity = true;

            // Add random initial velocity
            rb.velocity = new Vector3(
                Random.Range(-1f, 1f),
                0f,
                Random.Range(-1f, 1f)
            ) * 2f;

            // Set material properties
            SetPhysicsMaterial(obstacle);

            spawnedObjects.Add(obstacle);
        }
    }

    void PlaceRamps()
    {
        for (int i = 0; i < numRamps; i++)
        {
            // Create a ramp object
            GameObject ramp = GameObject.CreatePrimitive(PrimitiveType.Capsule);
            ramp.name = "Ramp_" + i;

            // Position the ramp
            Vector3 position = new Vector3(
                Random.Range(placementAreaPadding + 10, terrainWidth - placementAreaPadding - 10),
                5f,
                Random.Range(placementAreaPadding + 10, terrainLength - placementAreaPadding - 10)
            );

            ramp.transform.position = position;

            // Rotate to create an incline
            float angle = Random.Range(15f, 45f);
            ramp.transform.rotation = Quaternion.Euler(angle, Random.Range(0f, 360f), 0f);

            // Scale to make it ramp-like
            ramp.transform.localScale = new Vector3(0.5f, 5f, 3f);

            // Add rigidbody
            Rigidbody rb = ramp.AddComponent<Rigidbody>();
            rb.isKinematic = true; // Static ramp

            // Set material properties
            SetPhysicsMaterial(ramp);

            // Remove the collider and add a box collider instead for better physics
            DestroyImmediate(ramp.GetComponent<CapsuleCollider>());
            BoxCollider boxCollider = ramp.AddComponent<BoxCollider>();
            boxCollider.size = new Vector3(1f, 10f, 6f);

            spawnedObjects.Add(ramp);
        }
    }

    void SetPhysicsMaterial(GameObject obj)
    {
        PhysicMaterial material = defaultMaterial;
        if (material == null)
        {
            material = new PhysicMaterial("GeneratedMaterial");
            material.dynamicFriction = dynamicFriction;
            material.staticFriction = staticFriction;
            material.bounciness = bounciness;
        }

        // Apply material to all colliders in the object
        Collider[] colliders = obj.GetComponentsInChildren<Collider>();
        foreach (Collider collider in colliders)
        {
            collider.material = material;
        }
    }

    // Method to modify terrain height at a specific point (for dynamic terrain modification)
    public void ModifyTerrainHeight(Vector3 worldPosition, float heightDelta, float radius)
    {
        if (heightMap == null) return;

        Terrain terrain = GetComponent<Terrain>();
        if (terrain == null) return;

        TerrainData terrainData = terrain.terrainData;
        Vector3 terrainPos = terrain.GetPosition();

        // Convert world position to terrain coordinates
        float xPercent = (worldPosition.x - terrainPos.x) / terrainData.size.x;
        float zPercent = (worldPosition.z - terrainPos.z) / terrainData.size.z;

        int centerX = (int)(xPercent * terrainData.heightmapResolution);
        int centerZ = (int)(zPercent * terrainData.heightmapResolution);

        int radiusInSamples = (int)(radius * terrainData.heightmapResolution / terrainData.size.x);

        // Modify height within radius
        for (int x = Mathf.Max(0, centerX - radiusInSamples); x < Mathf.Min(terrainData.heightmapResolution, centerX + radiusInSamples); x++)
        {
            for (int z = Mathf.Max(0, centerZ - radiusInSamples); z < Mathf.Min(terrainData.heightmapResolution, centerZ + radiusInSamples); z++)
            {
                float distance = Mathf.Sqrt(Mathf.Pow(x - centerX, 2) + Mathf.Pow(z - centerZ, 2));
                if (distance <= radiusInSamples)
                {
                    float falloff = 1f - (distance / radiusInSamples);
                    heightMap[x, z] += heightDelta * falloff;
                    heightMap[x, z] = Mathf.Clamp01(heightMap[x, z]);
                }
            }
        }

        // Apply the modified heightmap
        terrainData.SetHeights(0, 0, heightMap);
    }
}
```

## Exercises

1. Create a Unity scene with realistic lighting and materials using PBR
2. Implement a camera sensor simulation that outputs RGB and depth images
3. Develop a LIDAR simulation system that outputs realistic point cloud data
4. Create an IMU simulation that provides accurate acceleration and rotation data
5. Build a procedural environment generator with physics-based objects

## References

- Unity Rendering Documentation: https://docs.unity3d.com/Manual/rendering-index.html
- Unity HDRP Documentation: https://docs.unity3d.com/Packages/com.unity.render-pipelines.high-definition@latest
- Unity Robotics Package: https://github.com/Unity-Technologies/Unity-Robotics-Hub
- Unity Perception Package: https://github.com/Unity-Technologies/Unity-Perception
- Physically-Based Rendering in Unity: https://docs.unity3d.com/Manual/MaterialsMenu.html