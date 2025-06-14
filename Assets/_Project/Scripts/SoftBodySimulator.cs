
using System.Collections.Generic;
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

        private struct Constraint
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

        private Mesh mesh;
        private List<Particle> particles;
        private List<Constraint> constraints;
        private List<int> indices;

        private int kernelIntegrate;
        private int kernelSolveConstraints;
        private int kernelUpdateMesh;

        private void Start()
        {
            Debug.Log("SoftBodySimulator: Starting initialization...");
            InitializeComputeShader();
            GenerateMesh();
            SetupBuffers();
            SetupRenderMaterial();

            // Diagnostic: Check if we have valid data
            Debug.Log($"Initialization complete. Particles: {particles?.Count}, Constraints: {constraints?.Count}");

            // Test: Manually move one particle to verify the system works
            if (particles != null && particles.Count > 0)
            {
                var testParticle = particles[0];
                Debug.Log($"First particle position: {testParticle.position}, invMass: {testParticle.invMass}");
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

            // Verify all kernels were found
            if (kernelIntegrate == -1 || kernelSolveConstraints == -1 || kernelUpdateMesh == -1)
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

            int res = settings.resolution;
            Vector3 spacing = new Vector3(
                settings.size.x / (res - 1),
                settings.size.y / (res - 1),
                settings.size.z / (res - 1)
            );

            // Generate particles in a 3D grid
            for (int x = 0; x < res; x++)
            {
                for (int y = 0; y < res; y++)
                {
                    for (int z = 0; z < res; z++)
                    {
                        Vector3 pos = new Vector3(
                            x * spacing.x - settings.size.x * 0.5f,
                            y * spacing.y - settings.size.y * 0.5f,
                            z * spacing.z - settings.size.z * 0.5f
                        );

                        Particle particle = new Particle
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

            // Generate constraints (springs between adjacent particles)
            for (int x = 0; x < res; x++)
            {
                for (int y = 0; y < res; y++)
                {
                    for (int z = 0; z < res; z++)
                    {
                        int index = x * res * res + y * res + z;

                        // Connect to adjacent particles
                        if (x < res - 1) AddConstraint(index, (x + 1) * res * res + y * res + z);
                        if (y < res - 1) AddConstraint(index, x * res * res + (y + 1) * res + z);
                        if (z < res - 1) AddConstraint(index, x * res * res + y * res + (z + 1));

                        // Diagonal constraints for stability
                        if (x < res - 1 && y < res - 1)
                            AddConstraint(index, (x + 1) * res * res + (y + 1) * res + z);
                        if (x < res - 1 && z < res - 1)
                            AddConstraint(index, (x + 1) * res * res + y * res + (z + 1));
                        if (y < res - 1 && z < res - 1)
                            AddConstraint(index, x * res * res + (y + 1) * res + (z + 1));
                    }
                }
            }

            // CRITICAL: Apply graph coloring to prevent race conditions
            ApplyGraphColoring();

            GenerateMeshTopology();
        }

        private void ApplyGraphColoring()
        {
            Debug.Log($"Applying graph coloring to {constraints.Count} constraints...");

            // Simple greedy graph coloring algorithm
            List<Constraint> coloredConstraints = new List<Constraint>();

            for (int i = 0; i < constraints.Count; i++)
            {
                Constraint constraint = constraints[i];

                // Find which colors are already used by constraints sharing particles
                HashSet<int> usedColors = new HashSet<int>();

                for (int j = 0; j < coloredConstraints.Count; j++)
                {
                    Constraint other = coloredConstraints[j];

                    // Check if constraints share particles
                    if (constraint.particleA == other.particleA || constraint.particleA == other.particleB ||
                        constraint.particleB == other.particleA || constraint.particleB == other.particleB)
                    {
                        usedColors.Add(other.colorGroup);
                    }
                }

                // Assign the smallest available color
                int color = 0;
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
            int maxColor = 0;
            foreach (var constraint in constraints)
            {
                maxColor = Mathf.Max(maxColor, constraint.colorGroup);
            }

            Debug.Log($"Graph coloring complete: {maxColor + 1} color groups needed");
        }

        private void AddConstraint(int a, int b)
        {
            float restLength = Vector3.Distance(particles[a].position, particles[b].position);

            Constraint constraint = new Constraint
            {
                particleA = a,
                particleB = b,
                restLength = restLength,
                compliance = settings.compliance,
                lambda = 0f, // Initialize accumulated lambda
                colorGroup = 0 // Will be set by graph coloring
            };

            constraints.Add(constraint);
        }

        private void GenerateMeshTopology()
        {
            indices.Clear();
            int res = settings.resolution;

            // Generate surface triangles (simplified cube faces)
            for (int x = 0; x < res - 1; x++)
            {
                for (int y = 0; y < res - 1; y++)
                {
                    for (int z = 0; z < res - 1; z++)
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
            int i000 = x * res * res + y * res + z;
            int i001 = x * res * res + y * res + (z + 1);
            int i010 = x * res * res + (y + 1) * res + z;
            int i011 = x * res * res + (y + 1) * res + (z + 1);
            int i100 = (x + 1) * res * res + y * res + z;
            int i101 = (x + 1) * res * res + y * res + (z + 1);
            int i110 = (x + 1) * res * res + (y + 1) * res + z;
            int i111 = (x + 1) * res * res + (y + 1) * res + (z + 1);

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
            particleBuffer =
                new ComputeBuffer(particles.Count, System.Runtime.InteropServices.Marshal.SizeOf<Particle>());
            constraintBuffer = new ComputeBuffer(constraints.Count,
                System.Runtime.InteropServices.Marshal.SizeOf<Constraint>());
            vertexBuffer = new ComputeBuffer(particles.Count, sizeof(float) * 3);
            indexBuffer = new ComputeBuffer(indices.Count, sizeof(int));

            // Upload initial data
            particleBuffer.SetData(particles);
            constraintBuffer.SetData(constraints);
            indexBuffer.SetData(indices);

            // Create mesh
            mesh = new Mesh();
            mesh.name = "SoftBody";

            Vector3[] vertices = new Vector3[particles.Count];
            for (int i = 0; i < particles.Count; i++)
            {
                vertices[i] = transform.InverseTransformPoint(particles[i].position);
            }

            mesh.vertices = vertices;
            mesh.triangles = indices.ToArray();
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();

            // Ensure MeshFilter exists and assign mesh
            MeshFilter meshFilter = GetComponent<MeshFilter>();
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
            MeshRenderer meshRenderer = GetComponent<MeshRenderer>();
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
            float targetDeltaTime = 1f / 120f; // 120 Hz physics
            float frameTime = Time.deltaTime;

            // Subdivide large frames into small steps
            int substeps = Mathf.CeilToInt(frameTime / targetDeltaTime);
            substeps = Mathf.Clamp(substeps, 1, 4); // Max 4 substeps per frame

            float substepDeltaTime = frameTime / substeps;

            for (int step = 0; step < substeps; step++)
            {
                bool isLastSubstep = (step == substeps - 1);
                SimulateSubstep(substepDeltaTime, isLastSubstep);
            }

            // Update mesh (async, won't block)
            UpdateMeshFromGPU();
        }

        private void SimulateSubstep(float deltaTime, bool isLastSubstep)
        {
            float floorY = FindFloorLevel();

            // Set compute shader parameters
            computeShader.SetFloat("deltaTime", deltaTime);
            computeShader.SetFloat("gravity", settings.gravity);
            computeShader.SetFloat("damping", settings.damping);
            computeShader.SetFloat("stiffness", settings.stiffness);
            computeShader.SetFloat("floorY", floorY);
            computeShader.SetVector("worldPosition", transform.position);
            computeShader.SetInt("particleCount", particles.Count);
            computeShader.SetInt("constraintCount", constraints.Count);

            // Bind buffers to all kernels
            computeShader.SetBuffer(kernelIntegrate, "particles", particleBuffer);
            computeShader.SetBuffer(kernelSolveConstraints, "particles", particleBuffer);
            computeShader.SetBuffer(kernelSolveConstraints, "constraints", constraintBuffer);
            computeShader.SetBuffer(kernelUpdateMesh, "particles", particleBuffer);
            computeShader.SetBuffer(kernelUpdateMesh, "vertices", vertexBuffer);

            // Integrate particles
            int threadGroups = Mathf.CeilToInt(particles.Count / 64f);
            if (threadGroups > 0)
            {
                computeShader.Dispatch(kernelIntegrate, threadGroups, 1, 1);
            }

            // CRITICAL: Solve constraints by color groups to prevent race conditions
            int maxColorGroup = GetMaxColorGroup();

            if (Time.frameCount % 60 == 0)
            {
                Debug.Log($"Solving {maxColorGroup + 1} color groups with compliance={settings.compliance}");
            }

            for (int colorGroup = 0; colorGroup <= maxColorGroup; colorGroup++)
            {
                // Set current color group - THIS WAS MISSING
                computeShader.SetInt("currentColorGroup", colorGroup);

                threadGroups = Mathf.CeilToInt(constraints.Count / 64f);
                if (threadGroups > 0)
                {
                    computeShader.Dispatch(kernelSolveConstraints, threadGroups, 1, 1);
                }
            }

            // Update mesh vertices (only on last substep to save bandwidth)
            if (isLastSubstep)
            {
                threadGroups = Mathf.CeilToInt(particles.Count / 64f);
                if (threadGroups > 0)
                {
                    computeShader.Dispatch(kernelUpdateMesh, threadGroups, 1, 1);
                }
            }
        }

        private int GetMaxColorGroup()
        {
            int maxColor = 0;
            foreach (var constraint in constraints)
            {
                maxColor = Mathf.Max(maxColor, constraint.colorGroup);
            }

            return maxColor;
        }

        // CPU fallback for testing and debugging
        private void UpdateCPU()
        {
            float deltaTime = Mathf.Min(Time.deltaTime, 0.02f);
            float floorY = FindFloorLevel();

            // Debug first particle before physics
            if (Time.frameCount % 30 == 0)
            {
                Debug.Log(
                    $"Before physics - First particle: pos={particles[0].position}, vel={particles[0].velocity}, invMass={particles[0].invMass}");
                Debug.Log($"Transform position: {transform.position}, Floor level: {floorY}");
            }

            // Integrate particles on CPU
            for (int i = 0; i < particles.Count; i++)
            {
                Particle p = particles[i];

                if (p.invMass <= 0) continue; // Skip pinned particles

                // Apply gravity force
                Vector3 gravityForce = Vector3.down * settings.gravity;
                p.force += gravityForce;

                // Simple physics integration
                Vector3 acceleration = p.force * p.invMass;
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
            for (int iter = 0; iter < settings.solverIterations; iter++)
            {
                for (int i = 0; i < constraints.Count; i++)
                {
                    var constraint = constraints[i];
                    Particle pA = particles[constraint.particleA];
                    Particle pB = particles[constraint.particleB];

                    Vector3 delta = pB.position - pA.position;
                    float currentLength = delta.magnitude;

                    if (currentLength > 0.001f) // Avoid division by zero
                    {
                        Vector3 direction = delta / currentLength;
                        float violation = currentLength - constraint.restLength;

                        float totalInvMass = pA.invMass + pB.invMass;
                        if (totalInvMass > 0)
                        {
                            float constraintMass = 1f / totalInvMass;
                            float lambda = -violation * constraintMass * settings.stiffness * 0.01f;

                            Vector3 correction = lambda * direction;

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

            // Update mesh directly - CRITICAL: Don't use transform space conversion yet
            Vector3[] vertices = new Vector3[particles.Count];
            Vector3 centerOffset = Vector3.zero;

            // Calculate center of mass for proper positioning
            for (int i = 0; i < particles.Count; i++)
            {
                centerOffset += particles[i].position;
            }

            centerOffset /= particles.Count;

            // Update transform position to follow center of mass
            transform.position = centerOffset;

            // Convert particle world positions to local mesh coordinates
            for (int i = 0; i < particles.Count; i++)
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
            Vector3[] vertices = new Vector3[particles.Count];
            Vector3 centerOffset = Vector3.zero;
            Vector3[] worldPositions = new Vector3[particles.Count];

            // First pass: read positions and check validity
            for (int i = 0; i < particles.Count; i++)
            {
                Vector3 worldPos = new Vector3(
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
            for (int i = 0; i < particles.Count; i++)
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
            for (int i = 0; i < particles.Count; i++)
            {
                Particle p = particles[i];
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
            for (int i = 0; i < particles.Count; i++)
            {
                float distance = Vector3.Distance(particles[i].position, position);
                if (distance < radius)
                {
                    float falloff = 1f - (distance / radius);
                    Particle p = particles[i];
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
            for (int i = 0; i < particles.Count; i++)
            {
                float distance = Vector3.Distance(particles[i].position, position);
                if (distance < radius)
                {
                    Particle p = particles[i];
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
                Vector3 originalPos = testParticle.position;
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
    }
}