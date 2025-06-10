using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Codex.Input;
using SoftBody.Scripts.Models;
using UnityEngine;

[RequireComponent(typeof(MeshFilter))]
public class SoftBodyGPU : MonoBehaviour
{

    // --- Shader Property IDs (for performance) ---
    private static readonly int Gravity = Shader.PropertyToID("_Gravity");
    private static readonly int GroundHeight = Shader.PropertyToID("_GroundHeight");
    private static readonly int Damping = Shader.PropertyToID("_Damping");
    private static readonly int DeltaTime = Shader.PropertyToID("_DeltaTime");
    private static readonly int ParticleCount = Shader.PropertyToID("_ParticleCount");
    private static readonly int ConstraintCount = Shader.PropertyToID("_ConstraintCount");
    private static readonly int DistanceCompliance = Shader.PropertyToID("_DistanceCompliance");
    private static readonly int Particles = Shader.PropertyToID("particles");
    private static readonly int Constraints = Shader.PropertyToID("constraints");
    private static readonly int LagrangeMultipliers = Shader.PropertyToID("lagrangeMultipliers");
    private static readonly int DistanceConstraintCount = Shader.PropertyToID("_DistanceConstraintCount");
    private static readonly int BendingCompliance = Shader.PropertyToID("_BendingCompliance");


    // --- Inspector Fields ---
    [Header("Compute Shader")] [SerializeField]
    private ComputeShader computeShader;

    [Header("Simulation Parameters")] [Range(1, 200)]
    public int substeps = 15;

    [Range(1, 25)] public int solverIterations = 15; // Inner loop for stiffness
    public Vector3 gravity = new(0, -9.81f, 0);
    [Range(0, 1f)] public float damping = 0.05f;

    [Header("Constraints")] [Tooltip("Compliance of the distance constraints. Lower is stiffer.")]
    public float distanceCompliance = 0f;

    [Tooltip("Compliance of the bending constraints. Resists folding.")]
    public float bendingCompliance = 0.05f;

    [Header("Collision")] public float groundHeight = 0.0f;
    [Header("Debug")] [SerializeField] private InputReaderSO inputReader;
    [SerializeField] private bool enableDebug = false; // Enable debug mode for additional logging

    // --- Private Member Variables ---
    private MeshFilter _meshFilter;
    private Mesh _simulationMesh;
    private int[] _originalIndexMap; 
    private ComputeBuffer _particleBuffer, _distanceConstraintBuffer, _lagrangeBuffer;
    private Particle[] _particles; // Used for initialization and mesh updates
    private List<DistanceConstraint> _distanceConstraints;
    private float[] _zeroData; // For clearing the Lagrange buffer
    private int _particleCount, _constraintCount;
    private int _predictKernel, _solveDistancesKernel, _updateStateKernel, _dampingKernel;
    private Vector3[] _verticesForMeshUpdate;
    private Particle[] _initialParticleState;
    private int _numDistanceConstraints;

    private void OnEnable()
    {
        inputReader.Test1PressedEvent += RestartSimulation;
    }

    private void OnDisable()
    {
        inputReader.Test1PressedEvent -= RestartSimulation;
    }

    private void Awake()
    {
        _meshFilter = GetComponent<MeshFilter>();
    }

    private void Start()
{
    WeldMeshVertices(out _simulationMesh, out _originalIndexMap);
    
    InitializeData();
    
    _verticesForMeshUpdate = new Vector3[_meshFilter.mesh.vertexCount];
    for (var i = 0; i < _verticesForMeshUpdate.Length; i++)
    {
        _verticesForMeshUpdate[i] = _simulationMesh.vertices[_originalIndexMap[i]];
    }
    _meshFilter.mesh.vertices = _verticesForMeshUpdate;
    
    _particleBuffer = new ComputeBuffer(_particleCount, Marshal.SizeOf(typeof(Particle)));
    _particleBuffer.SetData(_particles);

    // Constraint Buffer: Holds all distance and bending constraints combined.
    _distanceConstraintBuffer = new ComputeBuffer(_constraintCount, Marshal.SizeOf(typeof(DistanceConstraint)));
    _distanceConstraintBuffer.SetData(_distanceConstraints);

    // Lagrange Buffer: Used by the XPBD solver. Needs to be cleared each frame.
    _lagrangeBuffer = new ComputeBuffer(_constraintCount, sizeof(float));
    _zeroData = new float[_constraintCount]; // Pre-allocate a zeroed array for fast clearing.
    _lagrangeBuffer.SetData(_zeroData);
    
    _predictKernel = computeShader.FindKernel("PredictPositions");
    _solveDistancesKernel = computeShader.FindKernel("SolveDistances");
    _updateStateKernel = computeShader.FindKernel("UpdateState");
    _dampingKernel = computeShader.FindKernel("ApplyDamping");
    
    // Predict Kernel
    computeShader.SetBuffer(_predictKernel, Particles, _particleBuffer);

    // Solver Kernel
    computeShader.SetBuffer(_solveDistancesKernel, Particles, _particleBuffer);
    computeShader.SetBuffer(_solveDistancesKernel, Constraints, _distanceConstraintBuffer);
    computeShader.SetBuffer(_solveDistancesKernel, LagrangeMultipliers, _lagrangeBuffer);
    
    computeShader.SetBuffer(_updateStateKernel, Particles, _particleBuffer);
    computeShader.SetBuffer(_dampingKernel, Particles, _particleBuffer);

        if (enableDebug)
        {
            Debug.Log($"substeps: {substeps}");
            Debug.Log($"solverIterations: {solverIterations}");
            Debug.Log($"Gravity: {gravity}");
            Debug.Log($"GroundHeight: {groundHeight}");
            Debug.Log($"Damping: {damping}");
            Debug.Log($"DistanceCompliance: {distanceCompliance}");
            Debug.Log($"BendingCompliance: {bendingCompliance}");
            Debug.Log("----------------------------------------");
            Debug.Log($"Initialized particles: {_initialParticleState.Length}");
            Debug.Log($"Initialized constraints: {_constraintCount}");
            Debug.Log($"Distance constraints: {_numDistanceConstraints}");
            Debug.Log($"Particle buffer size: {_particleBuffer.count}");
            Debug.Log($"Distance constraint buffer size: {_distanceConstraintBuffer.count}");
            Debug.Log($"Lagrange buffer size: {_lagrangeBuffer.count}");
            Debug.Log("Simulation initialized successfully.");
        }
    }

    private void RestartSimulation()
    {
        if (_initialParticleState == null || _particleBuffer == null)
        {
            Debug.LogWarning("Simulation cannot be restarted before it has been initialized.");
            return;
        }

        Debug.Log("Restarting simulation (Optimized)...");

        Array.Copy(_initialParticleState, _particles, _particleCount);

        _particleBuffer.SetData(_particles);

        if (_lagrangeBuffer != null)
        {
            _lagrangeBuffer.SetData(_zeroData);
        }
    }

    private void FixedUpdate()
{
    if (_particleBuffer == null || _distanceConstraintBuffer == null || _lagrangeBuffer == null)
    {
        return;
    }

    // --- 2. CALCULATE CONSTANTS FOR THE FRAME ---

    var substepDeltaTime = Time.fixedDeltaTime / substeps;
    var particleThreadGroups = Mathf.CeilToInt(_particleCount / 64.0f);
    var constraintThreadGroups = Mathf.CeilToInt(_constraintCount / 64.0f);

    // --- 3. SET UNIFORMS FOR THE COMPUTE SHADER ---
    // These values are constant for all substeps in this frame.
    computeShader.SetInt(ParticleCount, _particleCount);
    computeShader.SetInt(ConstraintCount, _constraintCount);
    computeShader.SetInt(DistanceConstraintCount, _numDistanceConstraints);
    computeShader.SetFloat(DeltaTime, substepDeltaTime);
    computeShader.SetVector(Gravity, gravity);
    computeShader.SetFloat(GroundHeight, groundHeight);
    computeShader.SetFloat(Damping, damping);
    computeShader.SetFloat(DistanceCompliance, distanceCompliance);
    computeShader.SetFloat(BendingCompliance, bendingCompliance);

    // --- 4. THE FULLY GPU-BASED SIMULATION LOOP ---
    for (var i = 0; i < substeps; i++)
    {
        // For XPBD, we reset the Lagrange multipliers before the solver runs.
        _lagrangeBuffer.SetData(_zeroData);
        
        computeShader.Dispatch(_predictKernel, particleThreadGroups, 1, 1);
        for (var j = 0; j < solverIterations; j++)
        {
            computeShader.Dispatch(_solveDistancesKernel, constraintThreadGroups, 1, 1);
        }
        computeShader.Dispatch(_updateStateKernel, particleThreadGroups, 1, 1);
    }
    
    computeShader.Dispatch(_dampingKernel, particleThreadGroups, 1, 1);
    
    _particleBuffer.GetData(_particles);
    
    for (var i = 0; i < _verticesForMeshUpdate.Length; i++)
    {
        var simulatedIndex = _originalIndexMap[i];
        
        _verticesForMeshUpdate[i] = transform.InverseTransformPoint(_particles[simulatedIndex].position);
    }

    // Assign the entire modified vertex array back to the visual mesh. This is the correct way.
    _meshFilter.mesh.vertices = _verticesForMeshUpdate;

    // Recalculate normals and bounds for correct lighting and culling.
    _meshFilter.mesh.RecalculateBounds();
    _meshFilter.mesh.RecalculateNormals();
}

    private void InitializeData()
    {
        var initialVertices = _simulationMesh.vertices; 
        var triangles = _simulationMesh.triangles; 
        _particleCount = initialVertices.Length;

        _particles = new Particle[_particleCount];
        for (var i = 0; i < _particleCount; i++)
        {
            var worldPos = transform.TransformPoint(initialVertices[i]);
            _particles[i] = new Particle
            {
                position = worldPos,
                predictedPosition = worldPos,
                velocity = Vector3.zero,
                inverseMass = 1.0f
            };

            if (enableDebug)
            {
                Debug.Log(
                    $"Initialized Particle {i}: Position = {_particles[i].position}, InverseMass = {_particles[i].inverseMass}");
            }
        }

        _distanceConstraints = new List<DistanceConstraint>();
        var bendingConstraints = new List<DistanceConstraint>();

        var edgeToTrianglesMap = new Dictionary<Tuple<int, int>, List<int>>();

        //Pass 1: Populate the edge map
        for (var i = 0; i < triangles.Length; i += 3)
        {
            for (var j = 0; j < 3; j++)
            {
                var p1Idx = triangles[i + j];
                var p2Idx = triangles[i + (j + 1) % 3];

                // Sort indices to make the edge key unique
                var edge = new Tuple<int, int>(Mathf.Min(p1Idx, p2Idx), Mathf.Max(p1Idx, p2Idx));

                if (!edgeToTrianglesMap.ContainsKey(edge))
                {
                    edgeToTrianglesMap[edge] = new List<int>();
                }

                // The value is the index of the first vertex of the triangle (i)
                edgeToTrianglesMap[edge].Add(i);
            }
        }

        // Pass 2: Generate all constraints
        var addedEdges = new HashSet<Tuple<int, int>>();
        foreach (var (edge, tris) in edgeToTrianglesMap)
        {
            // Add the primary distance constraint for the edge
            if (addedEdges.Add(edge))  
            {
                _distanceConstraints.Add(new DistanceConstraint
                {
                    p1 = edge.Item1, p2 = edge.Item2,
                    restLength = Vector3.Distance(_particles[edge.Item1].position, _particles[edge.Item2].position)
                });
            }

            // If this edge is shared by two triangles, create a bending constraint
            if (tris.Count == 2)
            {
                // Find the two vertices that are NOT part of the shared edge
                var tri1StartIdx = tris[0];
                var tri2StartIdx = tris[1];
                int p3 = -1, p4 = -1;

                for (var i = 0; i < 3; i++)
                {
                    var vIdx = triangles[tri1StartIdx + i];
                    if (vIdx != edge.Item1 && vIdx != edge.Item2)
                    {
                        p3 = vIdx;
                        break;
                    }
                }

                for (var i = 0; i < 3; i++)
                {
                    var vIdx = triangles[tri2StartIdx + i];
                    if (vIdx != edge.Item1 && vIdx != edge.Item2)
                    {
                        p4 = vIdx;
                        break;
                    }
                }

                // If we found two valid outer vertices, create the constraint
                if (p3 != -1 && p4 != -1)
                {
                    bendingConstraints.Add(new DistanceConstraint
                    {
                        p1 = p3, p2 = p4,
                        restLength = Vector3.Distance(_particles[p3].position, _particles[p4].position)
                    });
                }
            }
        }

        // --- Final Step: Combine the lists and set the counts ---
        _numDistanceConstraints = _distanceConstraints.Count;
        _distanceConstraints.AddRange(bendingConstraints);
        _constraintCount = _distanceConstraints.Count;

        Debug.Log( $"Constraint Generation Complete: {_numDistanceConstraints} distance constraints, {bendingConstraints.Count} bending constraints. Total: {_constraintCount}");
    }
    
    private void WeldMeshVertices(out Mesh weldedMesh, out int[] mapToOriginal)
    {
        var originalMesh = _meshFilter.mesh;
        var originalVertices = originalMesh.vertices;
        var newVertices = new List<Vector3>();
        var oldToNewMap = new Dictionary<int, int>();
        mapToOriginal = new int[originalVertices.Length];

        for (int i = 0; i < originalVertices.Length; i++)
        {
            bool found = false;
            for (int j = 0; j < newVertices.Count; j++)
            {
                if (Vector3.Distance(newVertices[j], originalVertices[i]) < 0.0001f)
                {
                    oldToNewMap[i] = j;
                    mapToOriginal[i] = j;
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                newVertices.Add(originalVertices[i]);
                int newIndex = newVertices.Count - 1;
                oldToNewMap[i] = newIndex;
                mapToOriginal[i] = newIndex;
            }
        }
        
        var newTriangles = new int[originalMesh.triangles.Length];
        for (int i = 0; i < originalMesh.triangles.Length; i++)
        {
            int oldIndex = originalMesh.triangles[i];
            newTriangles[i] = oldToNewMap[oldIndex];
        }

        weldedMesh = new Mesh();
        weldedMesh.SetVertices(newVertices);
        weldedMesh.SetTriangles(newTriangles, 0);
        weldedMesh.RecalculateNormals();

        Debug.Log($"Mesh Welded: From {originalMesh.vertexCount} vertices to {newVertices.Count} unique vertices.");
    }
    
    private void OnDestroy()
    {
        if (_particleBuffer != null) _particleBuffer.Release();
        if (_distanceConstraintBuffer != null) _distanceConstraintBuffer.Release();
        if (_lagrangeBuffer != null) _lagrangeBuffer.Release();
    }
}