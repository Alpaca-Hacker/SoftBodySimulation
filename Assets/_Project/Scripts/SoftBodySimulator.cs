
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;

namespace SoftBody.Scripts
{
    public class SoftBodySimulator : MonoBehaviour
    {
        [SerializeField] private SoftBodySettings settings = new SoftBodySettings();
        [SerializeField] private ComputeShader computeShader;
        [SerializeField] private Material renderMaterial;

        private struct Particle
        {
            public Vector3 position;
            public Vector3 velocity;
            public Vector3 force;
            public float invMass;
        }

        public struct Constraint
        {
            public int particleA;
            public int particleB;
            public float restLength;
            public float compliance;
            public float lambda; // XPBD accumulated lambda
            public int colorGroup; // Graph coloring group
        }

        private ComputeBuffer particleBuffer;
        private ComputeBuffer constraintBuffer;
        private ComputeBuffer vertexBuffer;
        private ComputeBuffer indexBuffer;
        private ComputeBuffer debugBuffer;

        private Mesh mesh;
        private List<Particle> particles;
        private List<Constraint> constraints;
        private List<int> indices;

        private int kernelIntegrate;
        private int kernelSolveConstraints;
        private int kernelUpdateMesh;
        private int kernelDecayLambdas;
        private int kernelComputeDiagnostics;
        private int kernelApplyFloorConstraint;

        private void Start()
        {
            try
            {
                Debug.Log("SoftBodySimulator: Starting initialization...");
                InitializeComputeShader();
                GenerateMesh();
                SetupBuffers();
                SetupRenderMaterial();
        
                // Diagnostic: Check if we have valid data
                Debug.Log($"Initialization complete. Particles: {particles?.Count}, Constraints: {constraints?.Count}");
                settings.LogSettings();
        
                // Test: Manually move one particle to verify the system works
                if (particles != null && particles.Count > 0)
                {
                    var testParticle = particles[0];
                    Debug.Log($"First particle position: {testParticle.position}, invMass: {testParticle.invMass}");
                }
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Initialization failed: {e.Message}\n{e.StackTrace}");
                // Ensure we're in a safe state
                settings.useCPUFallback = true;
            }
        }

        private void InitializeComputeShader()
        {
            if (computeShader == null)
            {
                Debug.LogError("Compute Shader not assigned! Please assign the XPBDSoftBody compute shader.");
                return;
            }

            kernelIntegrate = computeShader.FindKernel("IntegrateParticles");
            kernelSolveConstraints = computeShader.FindKernel("SolveConstraints");
            kernelUpdateMesh = computeShader.FindKernel("UpdateMesh");
            kernelDecayLambdas = computeShader.FindKernel("DecayLambdas");
            kernelComputeDiagnostics = computeShader.FindKernel("ComputeDiagnostics");
            kernelApplyFloorConstraint = computeShader.FindKernel("ApplyFloorConstraint");

            // Verify all kernels were found
            if (kernelIntegrate == -1 || kernelSolveConstraints == -1 || kernelUpdateMesh == -1 || kernelDecayLambdas == -1)
            {
                Debug.LogError(
                    "Could not find required compute shader kernels! Make sure the compute shader has IntegrateParticles, SolveConstraints, and UpdateMesh kernels.");
            }
            else
            {
                Debug.Log("Compute shader kernels found successfully.");
            }
        }

        private void GenerateMesh()
        {
            particles = new List<Particle>();
            constraints = new List<Constraint>();
            indices = new List<int>();

            var res = settings.resolution;
            var spacing = new Vector3(
                settings.size.x / (res - 1),
                settings.size.y / (res - 1),
                settings.size.z / (res - 1)
            );

            // Generate particles in a 3D grid
            for (var x = 0; x < res; x++)
            {
                for (var y = 0; y < res; y++)
                {
                    for (var z = 0; z < res; z++)
                    {
                        var pos = new Vector3(
                            x * spacing.x - settings.size.x * 0.5f,
                            y * spacing.y - settings.size.y * 0.5f,
                            z * spacing.z - settings.size.z * 0.5f
                        );

                        var particle = new Particle
                        {
                            position = transform.TransformPoint(pos),
                            velocity = Vector3.zero,
                            force = Vector3.zero,
                            invMass = 1f / settings.mass
                        };

                        particles.Add(particle);
                    }
                }
            }
            
            //  AddDistanceConstraints();
            //   AddVolumeConstraints();
            GenerateStructuralConstraints();  // Main edges
            GenerateShearConstraints();       // Face diagonals  
            GenerateBendConstraints();        // Volume diagonals
            ApplyGraphColoring();
            GenerateMeshTopology();
        }

        private void AddDistanceConstraints()
        {
            var resolution = settings.resolution;
            for (var x = 0; x < resolution; x++)
            {
                for (var y = 0; y < resolution; y++)
                {
                    for (var z = 0; z < resolution; z++)
                    {
                        var index = x * resolution * resolution + y * resolution + z;

                        // Connect to adjacent particles
                        if (x < resolution - 1)
                            AddConstraint(index, (x + 1) * resolution * resolution + y * resolution + z);
                        if (y < resolution - 1)
                            AddConstraint(index, x * resolution * resolution + (y + 1) * resolution + z);
                        if (z < resolution - 1)
                            AddConstraint(index, x * resolution * resolution + y * resolution + (z + 1));

                        // Diagonal constraints for stability
                        if (x < resolution - 1 && y < resolution - 1)
                            AddConstraint(index, (x + 1) * resolution * resolution + (y + 1) * resolution + z);
                        if (x < resolution - 1 && z < resolution - 1)
                            AddConstraint(index, (x + 1) * resolution * resolution + y * resolution + (z + 1));
                        if (y < resolution - 1 && z < resolution - 1)
                            AddConstraint(index, x * resolution * resolution + (y + 1) * resolution + (z + 1));
                    }
                }
            }
            Debug.Log($"Added distance constraints. Constraints: {constraints.Count}");
        }
        
        private void AddVolumeConstraints()
            {
                // Add long-range constraints across diagonals
                var res = settings.resolution;

                for (var x = 0; x < res - 1; x++)
                {
                    for (var y = 0; y < res - 1; y++)
                    {
                        for (var z = 0; z < res - 1; z++)
                        {
                            // Add cube diagonal constraints
                            var i000 = x * res * res + y * res + z;
                            var i111 = (x + 1) * res * res + (y + 1) * res + (z + 1);
                            AddConstraint(i000, i111);

                            // Add face diagonals
                            var i001 = x * res * res + y * res + (z + 1);
                            var i110 = (x + 1) * res * res + (y + 1) * res + z;
                            AddConstraint(i001, i110);
                        }
                    }
                }

                Debug.Log($"Added volume preservation constraints. Total constraints: {constraints.Count}");
            }
        
        private void GenerateStructuralConstraints()
        {
            var res = settings.resolution;
    
            // Edge constraints (what you have now)
            for (var x = 0; x < res; x++)
            {
                for (var y = 0; y < res; y++)
                {
                    for (var z = 0; z < res; z++)
                    {
                        var index = x * res * res + y * res + z;
                
                        if (x < res - 1) AddConstraint(index, (x + 1) * res * res + y * res + z, settings.structuralCompliance);
                        if (y < res - 1) AddConstraint(index, x * res * res + (y + 1) * res + z, settings.structuralCompliance);
                        if (z < res - 1) AddConstraint(index, x * res * res + y * res + (z + 1), settings.structuralCompliance);
                    }
                }
            }
        }

        private void GenerateShearConstraints()
        {
            var res = settings.resolution;

            // Face diagonals - prevent shearing
            for (var x = 0; x < res - 1; x++)
            {
                for (var y = 0; y < res - 1; y++)
                {
                    for (var z = 0; z < res - 1; z++)
                    {
                        // XY face diagonals
                        AddConstraint(
                            x * res * res + y * res + z,
                            (x + 1) * res * res + (y + 1) * res + z,
                            settings.shearCompliance
                        );

                        // XZ face diagonals  
                        AddConstraint(
                            x * res * res + y * res + z,
                            (x + 1) * res * res + y * res + (z + 1),
                            settings.shearCompliance
                        );

                        // YZ face diagonals
                        AddConstraint(
                            x * res * res + y * res + z,
                            x * res * res + (y + 1) * res + (z + 1),
                            settings.shearCompliance
                        );
                    }
                }
            }
        }
        private void GenerateBendConstraints()
        {
            var res = settings.resolution;
    
            // Long-range constraints for volume preservation
            for (var x = 0; x < res - 1; x++)
            {
                for (var y = 0; y < res - 1; y++)
                {
                    for (var z = 0; z < res - 1; z++)
                    {
                        // Cube diagonals
                        AddConstraint(
                            x * res * res + y * res + z,
                            (x + 1) * res * res + (y + 1) * res + (z + 1),
                            settings.bendCompliance
                        );
                    }
                }
            }
        }
        
        private void ApplyGraphColoring()
        {
            // Initialize all constraints with color group 0 as fallback
            for (var i = 0; i < constraints.Count; i++)
            {
                var c = constraints[i];
                c.colorGroup = 0;
                constraints[i] = c;
            }
    
            try
            {
                // Try graph clustering
                var clusters = GraphClustering.CreateClusters(constraints, particles.Count);
                GraphClustering.ColorClusters(clusters, constraints);
            }
            catch (System.Exception e)
            {
                Debug.LogWarning($"Graph clustering failed: {e.Message}, using naive coloring");
                ApplyNaiveGraphColoring();
            }
        }
        
        
        private void ApplyNaiveGraphColoring()
        {
            Debug.Log($"Applying naive graph coloring to {constraints.Count} constraints...");

            // Simple greedy graph coloring algorithm
            var coloredConstraints = new List<Constraint>();

            for (var i = 0; i < constraints.Count; i++)
            {
                var constraint = constraints[i];

                // Find which colors are already used by constraints sharing particles
                var usedColors = new HashSet<int>();

                for (var j = 0; j < coloredConstraints.Count; j++)
                {
                    var other = coloredConstraints[j];

                    // Check if constraints share particles
                    if (constraint.particleA == other.particleA || constraint.particleA == other.particleB ||
                        constraint.particleB == other.particleA || constraint.particleB == other.particleB)
                    {
                        usedColors.Add(other.colorGroup);
                    }
                }

                // Assign the smallest available color
                var color = 0;
                while (usedColors.Contains(color))
                {
                    color++;
                }

                constraint.colorGroup = color;
                coloredConstraints.Add(constraint);
            }

            // Update the constraints list
            constraints = coloredConstraints;

            // Count colors used
            var maxColor = 0;
            foreach (var constraint in constraints)
            {
                maxColor = Mathf.Max(maxColor, constraint.colorGroup);
            }

            Debug.Log($"Graph coloring complete: {maxColor + 1} color groups needed");
        }
        
        

        private void AddConstraint(int a, int b, float compliance = float.PositiveInfinity)
        {
            if (float.IsPositiveInfinity(compliance))
            {
                compliance = settings.compliance; // Use default compliance if not specified
            }
            var restLength = Vector3.Distance(particles[a].position, particles[b].position);
    
            if (restLength < 0.001f) return;

            var constraint = new Constraint
            {
                particleA = a,
                particleB = b,
                restLength = restLength,
                compliance = compliance, // Scale compliance
                lambda = 0f,
                colorGroup = 0
            };

            constraints.Add(constraint);
        }

        private void GenerateMeshTopology()
        {
            indices.Clear();
            var res = settings.resolution;

            // Generate surface triangles (simplified cube faces)
            for (var x = 0; x < res - 1; x++)
            {
                for (var y = 0; y < res - 1; y++)
                {
                    for (var z = 0; z < res - 1; z++)
                    {
                        // Only render surface faces
                        if (x == 0 || x == res - 2 || y == 0 || y == res - 2 || z == 0 || z == res - 2)
                        {
                            AddCubeFace(x, y, z, res);
                        }
                    }
                }
            }
        }

        private void AddCubeFace(int x, int y, int z, int res)
        {
            var i000 = x * res * res + y * res + z;
            var i001 = x * res * res + y * res + (z + 1);
            var i010 = x * res * res + (y + 1) * res + z;
            var i011 = x * res * res + (y + 1) * res + (z + 1);
            var i100 = (x + 1) * res * res + y * res + z;
            var i101 = (x + 1) * res * res + y * res + (z + 1);
            var i110 = (x + 1) * res * res + (y + 1) * res + z;
            var i111 = (x + 1) * res * res + (y + 1) * res + (z + 1);

            // Add triangles for visible faces
            if (x == 0) AddQuad(i000, i010, i011, i001); // Left face
            if (x == res - 2) AddQuad(i100, i101, i111, i110); // Right face
            if (y == 0) AddQuad(i000, i001, i101, i100); // Bottom face
            if (y == res - 2) AddQuad(i010, i110, i111, i011); // Top face
            if (z == 0) AddQuad(i000, i100, i110, i010); // Front face
            if (z == res - 2) AddQuad(i001, i011, i111, i101); // Back face
        }

        private void AddQuad(int a, int b, int c, int d)
        {
            // Triangle 1 (counter-clockwise for correct normals)
            indices.Add(a);
            indices.Add(c);
            indices.Add(b);

            // Triangle 2 (counter-clockwise for correct normals)
            indices.Add(a);
            indices.Add(d);
            indices.Add(c);
        }

        private void SetupBuffers()
        {
            // Create compute buffers
            particleBuffer = new ComputeBuffer(particles.Count, System.Runtime.InteropServices.Marshal.SizeOf<Particle>());
            constraintBuffer = new ComputeBuffer(constraints.Count, System.Runtime.InteropServices.Marshal.SizeOf<Constraint>());
            vertexBuffer = new ComputeBuffer(particles.Count, sizeof(float) * 3);
            indexBuffer = new ComputeBuffer(indices.Count, sizeof(int));
            debugBuffer = new ComputeBuffer(1, sizeof(float) * 4);

            // Upload initial data
            particleBuffer.SetData(particles);
            constraintBuffer.SetData(constraints);
            indexBuffer.SetData(indices);

            // Create mesh
            mesh = new Mesh();
            mesh.name = "SoftBody";

            var vertices = new Vector3[particles.Count];
            for (var i = 0; i < particles.Count; i++)
            {
                vertices[i] = transform.InverseTransformPoint(particles[i].position);
            }

            mesh.vertices = vertices;
            mesh.triangles = indices.ToArray();
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();

            // Ensure MeshFilter exists and assign mesh
            var meshFilter = GetComponent<MeshFilter>();
            if (meshFilter == null)
            {
                meshFilter = gameObject.AddComponent<MeshFilter>();
            }

            meshFilter.mesh = mesh;

            Debug.Log($"Soft body initialized with {particles.Count} particles and {constraints.Count} constraints");
            Debug.Log(
                $"Constraint buffer size: {System.Runtime.InteropServices.Marshal.SizeOf<Constraint>()} bytes per constraint");
        }

        private void SetupRenderMaterial()
        {
            var meshRenderer = GetComponent<MeshRenderer>();
            if (meshRenderer == null)
            {
                meshRenderer = gameObject.AddComponent<MeshRenderer>();
            }

            if (renderMaterial == null)
            {
                renderMaterial = new Material(Shader.Find("Standard"));
                renderMaterial.color = Color.cyan;
                // Remove metallic and smoothness - not all materials have these
            }

            meshRenderer.material = renderMaterial;
        }

        private void Update()
        {
            if (settings.useCPUFallback)
            {
                UpdateCPU();
                return;
            }

            if (computeShader == null)
            {
                Debug.LogError("Compute Shader not assigned to SoftBodySimulator!");
                return;
            }

            if (particleBuffer == null)
            {
                Debug.LogError("Particle buffer not initialized!");
                return;
            }

            // Use small timesteps for stability (key XPBD principle)
            var targetDeltaTime = 1f / 120f; // 120 Hz physics
            var frameTime = Time.deltaTime;

            // Subdivide large frames into small steps
            var substeps = Mathf.CeilToInt(frameTime / targetDeltaTime);
            substeps = Mathf.Clamp(substeps, 1, 4); // Max 4 substeps per frame

            var substepDeltaTime = frameTime / substeps;

            for (var step = 0; step < substeps; step++)
            {
                var isLastSubstep = (step == substeps - 1);
                SimulateSubstep(substepDeltaTime, isLastSubstep);
            }

            // Update mesh (async, won't block)
            UpdateMeshFromGPU();
        }

        private void SimulateSubstep(float deltaTime, bool isLastSubstep)
        {
            var floorY = FindFloorLevel();

            // Set compute shader parameters
            computeShader.SetFloat("deltaTime", deltaTime);
            computeShader.SetFloat("gravity", settings.gravity);
            computeShader.SetFloat("damping", settings.damping);
            computeShader.SetFloat("stiffness", settings.stiffness);
            computeShader.SetFloat("floorY", floorY);
            computeShader.SetVector("worldPosition", transform.position);
            computeShader.SetInt("particleCount", particles.Count);
            computeShader.SetInt("constraintCount", constraints.Count);
            computeShader.SetFloat("lambdaDecay", settings.lambdaDecay);


            // Bind buffers to all kernels
            computeShader.SetBuffer(kernelIntegrate, "particles", particleBuffer);
            computeShader.SetBuffer(kernelSolveConstraints, "particles", particleBuffer);
            computeShader.SetBuffer(kernelSolveConstraints, "constraints", constraintBuffer);
            computeShader.SetBuffer(kernelUpdateMesh, "particles", particleBuffer);
            computeShader.SetBuffer(kernelUpdateMesh, "vertices", vertexBuffer);
            computeShader.SetBuffer(kernelDecayLambdas, "constraints", constraintBuffer);
            computeShader.SetBuffer(kernelComputeDiagnostics, "particles", particleBuffer);
            computeShader.SetBuffer(kernelComputeDiagnostics, "constraints", constraintBuffer);
            computeShader.SetBuffer(kernelApplyFloorConstraint, "particles", particleBuffer);
            computeShader.SetBuffer(kernelComputeDiagnostics, "debugBuffer", debugBuffer);
         

            // Integrate particles
            var constraintThreadGroups = Mathf.CeilToInt(constraints.Count / 64f);
            var particleThreadGroups = Mathf.CeilToInt(particles.Count / 64f);
            
            if (constraintThreadGroups > 0)
            {
                computeShader.Dispatch(kernelDecayLambdas, constraintThreadGroups, 1, 1);
            }
            
            if (particleThreadGroups > 0)
            {
                computeShader.Dispatch(kernelIntegrate, particleThreadGroups, 1, 1);
            }

            for (var iter = 0; iter < settings.solverIterations; iter++)
            {
                // Solve constraints by color groups to prevent race conditions
                var maxColorGroup = GetMaxColorGroup();

                if (Time.frameCount % 60 == 0 && settings.debugMode)
                {
                    Debug.Log($"Solving {maxColorGroup + 1} colour groups with compliance={settings.compliance}");
                }

                for (var colorGroup = 0; colorGroup <= maxColorGroup; colorGroup++)
                {
                    // Set current color group
                    computeShader.SetInt("currentColorGroup", colorGroup);
                    
                    if (constraintThreadGroups > 0)
                    {
                        computeShader.Dispatch(kernelSolveConstraints, constraintThreadGroups, 1, 1);
                    }
                }
                
                computeShader.Dispatch(kernelApplyFloorConstraint, particleThreadGroups, 1, 1);
            }

            // Update mesh vertices (only on last substep to save bandwidth)
            if (isLastSubstep)
            {
                if (particleThreadGroups > 0)
                {
                    computeShader.Dispatch(kernelUpdateMesh, particleThreadGroups, 1, 1);
                }
            }
            
            computeShader.Dispatch(kernelComputeDiagnostics, 1, 1, 1);
            
            if (Time.frameCount % 30 == 0 && settings.debugMode)
            {
                var debugData = new float[4];
                debugBuffer.GetData(debugData);
                Debug.Log($"Diagnostics - MaxVel: {debugData[0]:F3}, MaxError: {debugData[1]:F3}, " +
                          $"AvgLambda: {debugData[2]:F3}, GroundParticles: {debugData[3]}");
            }
        }

        private int GetMaxColorGroup()
        {
            return constraints.Max(c => c.colorGroup);
        }

        // CPU fallback for testing and debugging
        private void UpdateCPU()
        {
            var deltaTime = Mathf.Min(Time.deltaTime, 0.02f);
            var floorY = FindFloorLevel();

            // Debug first particle before physics
            if (Time.frameCount % 30 == 0)
            {
                Debug.Log(
                    $"Before physics - First particle: pos={particles[0].position}, vel={particles[0].velocity}, invMass={particles[0].invMass}");
                Debug.Log($"Transform position: {transform.position}, Floor level: {floorY}");
            }

            // Integrate particles on CPU
            for (var i = 0; i < particles.Count; i++)
            {
                var p = particles[i];

                if (p.invMass <= 0) continue; // Skip pinned particles

                // Apply gravity force
                var gravityForce = Vector3.down * settings.gravity;
                p.force += gravityForce;

                // Simple physics integration
                var acceleration = p.force * p.invMass;
                p.velocity += acceleration * deltaTime;

                // Apply damping
                p.velocity *= (1f - settings.damping);

                // Update position
                p.position += p.velocity * deltaTime;

                // Floor collision
                if (p.position.y < floorY)
                {
                    p.position.y = floorY;
                    if (p.velocity.y < 0)
                    {
                        p.velocity.y = -p.velocity.y * 0.3f; // Bounce
                    }

                    // Friction
                    p.velocity.x *= 0.9f;
                    p.velocity.z *= 0.9f;
                }

                // Reset forces for next frame
                p.force = Vector3.zero;
                particles[i] = p;
            }

            // Simple constraint solving (distance constraints)
            for (var iter = 0; iter < settings.solverIterations; iter++)
            {
                for (var i = 0; i < constraints.Count; i++)
                {
                    var constraint = constraints[i];
                    var pA = particles[constraint.particleA];
                    var pB = particles[constraint.particleB];

                    var delta = pB.position - pA.position;
                    var currentLength = delta.magnitude;

                    if (currentLength > 0.001f) // Avoid division by zero
                    {
                        var direction = delta / currentLength;
                        var violation = currentLength - constraint.restLength;

                        var totalInvMass = pA.invMass + pB.invMass;
                        if (totalInvMass > 0)
                        {
                            var constraintMass = 1f / totalInvMass;
                            var lambda = -violation * constraintMass * settings.stiffness * 0.01f;

                            var correction = lambda * direction;

                            if (pA.invMass > 0)
                            {
                                pA.position -= correction * pA.invMass;
                                particles[constraint.particleA] = pA;
                            }

                            if (pB.invMass > 0)
                            {
                                pB.position += correction * pB.invMass;
                                particles[constraint.particleB] = pB;
                            }
                        }
                    }
                }
            }

            // Update mesh directly
            var vertices = new Vector3[particles.Count];
            var centerOffset = Vector3.zero;

            // Calculate center of mass for proper positioning
            for (var i = 0; i < particles.Count; i++)
            {
                centerOffset += particles[i].position;
            }

            centerOffset /= particles.Count;

            // Update transform position to follow center of mass
            transform.position = centerOffset;

            // Convert particle world positions to local mesh coordinates
            for (var i = 0; i < particles.Count; i++)
            {
                vertices[i] = particles[i].position - centerOffset;
            }

            try
            {
                mesh.vertices = vertices;
                mesh.RecalculateNormals();
                mesh.RecalculateBounds();

                // Force mesh filter to update
                GetComponent<MeshFilter>().mesh = mesh;
            }
            catch (System.Exception e)
            {
                Debug.LogError($"CPU mesh update failed: {e.Message}");
            }

            // Debug after physics
            if (Time.frameCount % 30 == 0)
            {
                Debug.Log($"After physics - First particle: pos={particles[0].position}, vel={particles[0].velocity}");
                Debug.Log($"Mesh vertex[0]: {vertices[0]}, Center: {centerOffset}");
            }
        }

        private float FindFloorLevel()
        {
            // Raycast downward to find the floor
            RaycastHit hit;
            if (Physics.Raycast(transform.position, Vector3.down, out hit, 100f, settings.collisionLayers))
            {
                return hit.point.y;
            }

            // Default floor level if no collider found
            return -5f;
        }

        private UnityEngine.Rendering.AsyncGPUReadbackRequest readbackRequest;
        private bool isReadbackPending = false;

        private void UpdateMeshFromGPU()
        {
            if (mesh == null) return;

            // Don't start new readback if one is pending
            if (isReadbackPending)
            {
                // Check if readback is complete
                if (readbackRequest.done)
                {
                    isReadbackPending = false;

                    if (readbackRequest.hasError)
                    {
                        Debug.LogError("AsyncGPUReadback failed! Switching to CPU mode.");
                        settings.useCPUFallback = true;
                        return;
                    }

                    // Process the readback data
                    var data = readbackRequest.GetData<float>();
                    ProcessVertexData(data);
                }

                return; // Wait for current readback to complete
            }

            // Start new async readback
            readbackRequest = UnityEngine.Rendering.AsyncGPUReadback.Request(vertexBuffer);
            isReadbackPending = true;
        }

        private void ProcessVertexData(Unity.Collections.NativeArray<float> vertexData)
        {
            var vertices = new Vector3[particles.Count];
            var centerOffset = Vector3.zero;
            var worldPositions = new Vector3[particles.Count];

            // First pass: read positions and check validity
            for (var i = 0; i < particles.Count; i++)
            {
                var worldPos = new Vector3(
                    vertexData[i * 3],
                    vertexData[i * 3 + 1],
                    vertexData[i * 3 + 2]
                );

                // Check for invalid data
                if (float.IsNaN(worldPos.x) || float.IsNaN(worldPos.y) || float.IsNaN(worldPos.z) ||
                    float.IsInfinity(worldPos.x) || float.IsInfinity(worldPos.y) || float.IsInfinity(worldPos.z))
                {
                    Debug.LogWarning($"Invalid GPU data at particle {i}: {worldPos}");
                    settings.useCPUFallback = true;
                    return;
                }

                worldPositions[i] = worldPos;
                centerOffset += worldPos;
            }

            // Calculate center of mass
            centerOffset /= particles.Count;

            // Update transform to follow center of mass
            transform.position = centerOffset;

            // Convert to local coordinates
            for (var i = 0; i < particles.Count; i++)
            {
                vertices[i] = worldPositions[i] - centerOffset;
            }

            try
            {
                mesh.vertices = vertices;
                mesh.RecalculateNormals();
                mesh.RecalculateBounds();

                // Force mesh filter update
                GetComponent<MeshFilter>().mesh = mesh;
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Failed to update mesh: {e.Message}");
            }
        }

        private void ResetToInitialPositions()
        {
            // Reset particles to initial positions
            for (var i = 0; i < particles.Count; i++)
            {
                var p = particles[i];
                p.velocity = Vector3.zero;
                p.force = Vector3.zero;
                // Keep original position but reset physics state
                particles[i] = p;
            }

            if (particleBuffer != null)
            {
                particleBuffer.SetData(particles);
            }

            Debug.Log("Reset particles to initial state due to invalid data");
        }

        private void OnDestroy()
        {
            if (particleBuffer != null) particleBuffer.Release();
            if (constraintBuffer != null) constraintBuffer.Release();
            if (vertexBuffer != null) vertexBuffer.Release();
            if (indexBuffer != null) indexBuffer.Release();
            if (debugBuffer != null) debugBuffer.Release();
            if (mesh != null)
            {
                Destroy(mesh);
            }
            
        }

        private void OnValidate()
        {
            // Regenerate mesh when settings change in editor
            if (Application.isPlaying && particles != null)
            {
                GenerateMesh();
                SetupBuffers();
            }
        }

        // Public methods for designer interaction
        public void AddForce(Vector3 force, Vector3 position, float radius = 1f)
        {
            // Add external force to particles within radius
            for (var i = 0; i < particles.Count; i++)
            {
                var distance = Vector3.Distance(particles[i].position, position);
                if (distance < radius)
                {
                    var falloff = 1f - (distance / radius);
                    var p = particles[i];
                    p.force += force * falloff;
                    particles[i] = p;
                }
            }

            particleBuffer.SetData(particles);
            Debug.Log($"Applied force {force} to {particles.Count} particles");
        }

        public void SetPinned(Vector3 position, float radius = 0.5f, bool pinned = true)
        {
            // Pin/unpin particles within radius
            for (var i = 0; i < particles.Count; i++)
            {
                var distance = Vector3.Distance(particles[i].position, position);
                if (distance < radius)
                {
                    var p = particles[i];
                    p.invMass = pinned ? 0f : 1f / settings.mass;
                    particles[i] = p;
                }
            }

            particleBuffer.SetData(particles);
        }

        // Test method - call this to verify the system is working
        [ContextMenu("Test Physics System")]
        public void TestPhysicsSystem()
        {
            Debug.Log("=== Physics System Test ===");
            Debug.Log($"Compute Shader: {(computeShader != null ? "Assigned" : "NULL")}");
            Debug.Log($"Particle Buffer: {(particleBuffer != null ? "Valid" : "NULL")}");
            Debug.Log($"Particles Count: {particles?.Count ?? 0}");
            Debug.Log($"Constraints Count: {constraints?.Count ?? 0}");
            Debug.Log($"CPU Fallback Mode: {settings.useCPUFallback}");

            if (particles != null && particles.Count > 0)
            {
                // Manually modify first particle to test - make it very obvious
                var testParticle = particles[0];
                var originalPos = testParticle.position;
                testParticle.position += Vector3.up * 2.0f; // Move up 2 units
                testParticle.velocity = Vector3.zero; // Reset velocity
                particles[0] = testParticle;

                // Update buffer if using GPU
                if (!settings.useCPUFallback && particleBuffer != null)
                {
                    particleBuffer.SetData(particles);
                }

                Debug.Log($"Moved particle 0 from {originalPos} to {testParticle.position}");
                Debug.Log("Watch for mesh movement - it should fall from the new position!");
            }

            // Force a big downward force on all particles
            AddForce(Vector3.down * 50f, transform.position, 10f);

            // Test mesh update manually in CPU mode
            if (settings.useCPUFallback)
            {
                Debug.Log("CPU mode - manually updating mesh...");
                UpdateCPU();
            }
        }
        
        [ContextMenu("Test Single Thread Solving")]
        public void TestSingleThreadSolving()
        {
            // Temporarily disable graph coloring
            for (var i = 0; i < constraints.Count; i++)
            {
                var c = constraints[i];
                c.colorGroup = i; // Each constraint gets unique color
                constraints[i] = c;
            }
            constraintBuffer.SetData(constraints);
    
            Debug.Log($"Testing with {constraints.Count} sequential color groups");
           
        }
        
        [ContextMenu("Validate Constraint Data")]
        public void ValidateConstraintData()
        {
            var constraintData = new Constraint[constraints.Count];
            constraintBuffer.GetData(constraintData);
    
            var validConstraints = 0;
            for (var i = 0; i < constraintData.Length; i++)
            {
                var c = constraintData[i];
                if (c.particleA >= 0 && c.particleA < particles.Count &&
                    c.particleB >= 0 && c.particleB < particles.Count &&
                    c.restLength > 0)
                {
                    validConstraints++;
                }
        
                if (i < 5) // Log first 5 constraints
                {
                    Debug.Log($"Constraint {i}: A={c.particleA}, B={c.particleB}, " +
                              $"RestLength={c.restLength}, Compliance={c.compliance}, " +
                              $"Lambda={c.lambda}, ColorGroup={c.colorGroup}");
                }
            }
    
            Debug.Log($"Valid constraints: {validConstraints}/{constraints.Count}");
        }

        [ContextMenu("Reset Lambdas")]
        private void ResetLambdas()
        {
            var constraintData = new Constraint[constraints.Count];
            constraintBuffer.GetData(constraintData);

            for (var i = 0; i < constraintData.Length; i++)
            {
                constraintData[i].lambda = 0f;
            }

            constraintBuffer.SetData(constraintData);
        }

        [ContextMenu("Simple Two Particle Test")]
        public void SimpleTwoParticleTest()
        {
            // Create just 2 particles
            particles = new List<Particle>();
            constraints = new List<Constraint>();

            // Particle 0 at origin (fixed)
            particles.Add(new Particle
            {
                position = Vector3.zero,
                velocity = Vector3.zero,
                force = Vector3.zero,
                invMass = 0f // Fixed
            });

            // Particle 1 stretched away
            particles.Add(new Particle
            {
                position = new Vector3(2f, 0, 0), // Stretched
                velocity = Vector3.zero,
                force = Vector3.zero,
                invMass = 1f
            });

            // One constraint with rest length 1
            constraints.Add(new Constraint
            {
                particleA = 0,
                particleB = 1,
                restLength = 1f,
                compliance = 0.0f, // Perfectly stiff
                lambda = 0f,
                colorGroup = 0
            });

            SetupBuffers();
            Debug.Log("Simple test setup: 2 particles, 1 constraint");
        }
        
    }
}