using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Codex.Input;
using SoftBody.Scripts;
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
    private static readonly int SubstepDeltaTime = Shader.PropertyToID("_SubstepDeltaTime");
    private static readonly int Friction = Shader.PropertyToID("_Friction");


    // --- Inspector Fields ---
    [Header("Compute Shader")] 
    [SerializeField]
    private ComputeShader computeShader;
    
    [Header("Mesh Setup")]
    [SerializeField]
    private PrimitiveType meshType = PrimitiveType.FromMeshFilter;
    [SerializeField]
    [Tooltip("Size parameter used for procedural primitives.")]
    private float primitiveSize = 1.0f;

    [Header("Simulation Parameters")] [Range(1, 200)]
    [SerializeField]
    private int substeps = 15;
    [SerializeField]
    [Range(1, 25)] 
    private int solverIterations = 15; // Inner loop for stiffness
    [SerializeField]
    private Vector3 gravity = new(0, -9.81f, 0);
    [Range(0, 1f)] public float damping = 0.05f;

    [Header("Constraints")] [Tooltip("Compliance of the distance constraints. Lower is stiffer.")]
    [SerializeField]
    private float distanceCompliance = 0f;

    [Tooltip("Compliance of the bending constraints. Resists folding.")]
    [SerializeField]
    private float bendingCompliance = 0.05f;

    [Header("Collision")] 
    [SerializeField]
    private float groundHeight = 0.0f;
    [SerializeField]
    [Range(0, 1f)]
    private float friction = 0.2f;
    
    [Header("Debug")] 
    [SerializeField] private InputReaderSO inputReader;
    [SerializeField] private bool enableDebug = false;
    [SerializeField] private bool pinFirstParticle = false;
    [SerializeField] private int framesToDebug = 10;

    // --- Private Member Variables ---
    private MeshFilter _meshFilter;
    private Mesh _simulationMesh;
    private int[] _originalIndexMap; 
    private ComputeBuffer _particleBuffer, _distanceConstraintBuffer, _lagrangeBuffer;
    private Particle[] _particles; // Used for initialization and mesh updates
    private List<DistanceConstraint> _distanceConstraints;
    private float[] _zeroData; // For clearing the Lagrange buffer
    private int _particleCount, _constraintCount;
    private int _predictKernel, _solveDistancesKernel, _updateStateKernel, _solveCollisionsKernel, _finalizeSubstepKernel;
    private Vector3[] _verticesForMeshUpdate;
    private Particle[] _initialParticleState;
    private int _numDistanceConstraints;
    private int frameCounter = 0;

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
        
        switch (meshType)
        {
            case PrimitiveType.Plane:
                _meshFilter.mesh = MeshFactory.CreatePlane(primitiveSize);
                break;
            case PrimitiveType.Cube:
                _meshFilter.mesh = MeshFactory.CreateCube(primitiveSize);
                break;
            case PrimitiveType.FromMeshFilter:
                if (_meshFilter.mesh == null)
                {
                    Debug.LogError("MeshType is set to FromMeshFilter, but no mesh is assigned!", this);
                }
                break;
        }
    }

    private void Start()
{
    WeldMeshVertices(out _simulationMesh, out _originalIndexMap);

    InitializeData();
    
    // Create a backup of the initial state for restarting the simulation
    _initialParticleState = new Particle[_particleCount];
    Array.Copy(_particles, _initialParticleState, _particleCount);

    _verticesForMeshUpdate = new Vector3[_meshFilter.mesh.vertexCount];
    for (var i = 0; i < _verticesForMeshUpdate.Length; i++)
    {
        _verticesForMeshUpdate[i] = _simulationMesh.vertices[_originalIndexMap[i]];
    }

    _meshFilter.mesh.vertices = _verticesForMeshUpdate;

    // --- Buffer Creation (This is all correct) ---
    _particleBuffer = new ComputeBuffer(_particleCount, Marshal.SizeOf(typeof(Particle)));
    _particleBuffer.SetData(_particles);

    _distanceConstraintBuffer = new ComputeBuffer(_constraintCount, Marshal.SizeOf(typeof(DistanceConstraint)));
    _distanceConstraintBuffer.SetData(_distanceConstraints);

    _lagrangeBuffer = new ComputeBuffer(_constraintCount, sizeof(float));
    _zeroData = new float[_constraintCount];
    _lagrangeBuffer.SetData(_zeroData);

    // --- Kernel Finding (Cleaned up) ---
    _predictKernel = computeShader.FindKernel("PredictPositions");
    _solveDistancesKernel = computeShader.FindKernel("SolveDistances");
    _solveCollisionsKernel = computeShader.FindKernel("SolveCollisions");
    _finalizeSubstepKernel = computeShader.FindKernel("FinalizeSubstep");

    // --- Buffer Linking ---
    computeShader.SetBuffer(_predictKernel, Particles, _particleBuffer);
    computeShader.SetBuffer(_solveDistancesKernel, Particles, _particleBuffer);
    computeShader.SetBuffer(_solveCollisionsKernel, Particles, _particleBuffer);
    computeShader.SetBuffer(_finalizeSubstepKernel, Particles, _particleBuffer);

    // Link other buffers to the kernels that need them.
    computeShader.SetBuffer(_solveDistancesKernel, Constraints, _distanceConstraintBuffer);
    computeShader.SetBuffer(_solveDistancesKernel, LagrangeMultipliers, _lagrangeBuffer);

    // --- Debug Logging ---
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
        Debug.Log($"Particle Buffer: Size={_particleBuffer.count}, Stride={_particleBuffer.stride}");
        Debug.Log($"Constraint Buffer: Size={_distanceConstraintBuffer.count}, Stride={_distanceConstraintBuffer.stride}");
        Debug.Log($"Lagrange Buffer: Size={_lagrangeBuffer.count}, Stride={_lagrangeBuffer.stride}");
        Debug.Log("Simulation initialized successfully.");
    }
}

    public void RestartSimulation()
    {
        if (_initialParticleState == null || _particleBuffer == null)
        {
            Debug.LogWarning("Simulation cannot be restarted before it has been initialized.");
            return;
        }

        _particleBuffer.SetData(_initialParticleState);

        _lagrangeBuffer?.SetData(_zeroData);

        for (var i = 0; i < _verticesForMeshUpdate.Length; i++)
        {
            var simulatedIndex = _originalIndexMap[i];
            _verticesForMeshUpdate[i] = transform.InverseTransformPoint(_initialParticleState[simulatedIndex].position);
        }

        _meshFilter.mesh.vertices = _verticesForMeshUpdate;
        _meshFilter.mesh.RecalculateBounds();
        _meshFilter.mesh.RecalculateNormals();
        
        Debug.Log("Simulation Restarted.");

    }
    
private void FixedUpdate()
{
    if (_particleBuffer == null) return;

    // We only need the substep delta time for the main loop.
    var substepDt = Time.fixedDeltaTime / substeps;
    
    var particleThreadGroups = Mathf.CeilToInt(_particleCount / 8.0f);
    var constraintThreadGroups = Mathf.CeilToInt(_constraintCount / 8.0f);

    // --- SET UNIFORMS ---
    computeShader.SetFloat(SubstepDeltaTime, substepDt); 
    computeShader.SetInt(ParticleCount, _particleCount);
    computeShader.SetInt(ConstraintCount, _constraintCount);
    computeShader.SetInt(DistanceConstraintCount, _numDistanceConstraints);
    computeShader.SetVector(Gravity, gravity);
    computeShader.SetFloat(GroundHeight, groundHeight);
    computeShader.SetFloat(Damping, damping); 
    computeShader.SetFloat(Friction, friction); 
    computeShader.SetFloat(DistanceCompliance, distanceCompliance);
    computeShader.SetFloat(BendingCompliance, bendingCompliance);

    // --- THE CORRECT SIMULATION LOOP ---
    for (var i = 0; i < substeps; i++)
    {
        computeShader.Dispatch(_predictKernel, particleThreadGroups, 1, 1);
        _lagrangeBuffer.SetData(_zeroData);
        
        for (var j = 0; j < solverIterations; j++)
        {
            computeShader.Dispatch(_solveDistancesKernel, constraintThreadGroups, 1, 1);
        }
        
        computeShader.Dispatch(_solveCollisionsKernel, particleThreadGroups, 1, 1);
        computeShader.Dispatch(_finalizeSubstepKernel, particleThreadGroups, 1, 1);
    }
    
    // --- READ BACK DATA FOR VISUALS ---
    _particleBuffer.GetData(_particles);
    frameCounter++;
    for (var k = 0; k < _verticesForMeshUpdate.Length; k++)
    {
        var simulatedIndex = _originalIndexMap[k];
        _verticesForMeshUpdate[k] = transform.InverseTransformPoint(_particles[simulatedIndex].position);
       
        if (enableDebug)
        {
            if (frameCounter % framesToDebug == 0)
            {
                Debug.Log($"Particle {simulatedIndex}: Position = {_particles[simulatedIndex].position}");
            }
       
        }
    }

    _meshFilter.mesh.vertices = _verticesForMeshUpdate;
    _meshFilter.mesh.RecalculateBounds();
    _meshFilter.mesh.RecalculateNormals();
}

   private void InitializeData()
{
    // --- Particle Initialization ---
    var initialVertices = _simulationMesh.vertices;
    _particleCount = initialVertices.Length;
    _particles = new Particle[_particleCount];
    for (var i = 0; i < _particleCount; i++)
    {
        var worldPos = transform.TransformPoint(initialVertices[i]);
        
        var isPinned = pinFirstParticle && i == 0;
        var invMass = isPinned ? 0.0f : 1.0f;
        
        _particles[i] = new Particle
        {
            position = new Vector4(worldPos.x, worldPos.y, worldPos.z, invMass),
            predictedPosition = worldPos,
            old_position = worldPos,
        };
    }
    
    // --- CONSTRAINT GENERATION ---
    
    // Get triangles as uint array
    var intTriangles = _simulationMesh.triangles;
    var triangles = new uint[intTriangles.Length];
    for(var i = 0; i < intTriangles.Length; i++)
    {
        triangles[i] = (uint)intTriangles[i];
    }
    
    _distanceConstraints = new List<DistanceConstraint>();
    var bendingConstraints = new List<DistanceConstraint>();
    
    // This map stores an edge and the two vertices opposite to it.
    var edgeToOppositeVerts = new Dictionary<Tuple<uint, uint>, List<uint>>();

    // --- Pass 1: Find all edges and their opposite vertices ---
    for (uint i = 0; i < triangles.Length; i += 3)
    {
        var p0 = triangles[i + 0];
        var p1 = triangles[i + 1];
        var p2 = triangles[i + 2];

        // Create the three edges for this triangle
        var edge1 = new Tuple<uint, uint>(Math.Min(p0, p1), Math.Max(p0, p1));
        var edge2 = new Tuple<uint, uint>(Math.Min(p1, p2), Math.Max(p1, p2));
        var edge3 = new Tuple<uint, uint>(Math.Min(p2, p0), Math.Max(p2, p0));

        // Initialize dictionary entries if they don't exist
        if (!edgeToOppositeVerts.ContainsKey(edge1)) edgeToOppositeVerts[edge1] = new List<uint>();
        if (!edgeToOppositeVerts.ContainsKey(edge2)) edgeToOppositeVerts[edge2] = new List<uint>();
        if (!edgeToOppositeVerts.ContainsKey(edge3)) edgeToOppositeVerts[edge3] = new List<uint>();

        // For each edge, the opposite vertex is the one not in the edge
        edgeToOppositeVerts[edge1].Add(p2);
        edgeToOppositeVerts[edge2].Add(p0);
        edgeToOppositeVerts[edge3].Add(p1);
    }

    // --- Pass 2: Create distance and bending constraints from the map ---
    foreach (var (edge, oppositeVerts) in edgeToOppositeVerts)
    {
        // 1. Add the edge itself as a distance constraint
        _distanceConstraints.Add(new DistanceConstraint
        {
            p1 = edge.Item1,
            p2 = edge.Item2,
            restLength = Vector3.Distance(_particles[edge.Item1].position, _particles[edge.Item2].position)
        });

        // 2. If two triangles share this edge, there will be two opposite vertices.
        // Create a bending constraint between them.
        if (oppositeVerts.Count == 2)
        {
            bendingConstraints.Add(new DistanceConstraint
            {
                p1 = oppositeVerts[0],
                p2 = oppositeVerts[1],
                restLength = Vector3.Distance(_particles[oppositeVerts[0]].position, _particles[oppositeVerts[1]].position)
            });
        }
    }

    // --- Final Step: Combine lists and set counts ---
    _numDistanceConstraints = _distanceConstraints.Count;
    _distanceConstraints.AddRange(bendingConstraints);
    _constraintCount = _distanceConstraints.Count;

    if (enableDebug)
    {
        Debug.Log( $"Constraint Generation Complete: {_numDistanceConstraints} distance constraints, {bendingConstraints.Count} bending constraints. Total: {_constraintCount}");
    }
}
    
    private void WeldMeshVertices(out Mesh weldedMesh, out int[] mapToOriginal)
    {
        var originalMesh = _meshFilter.mesh;
        var originalVertices = originalMesh.vertices;
        var newVertices = new List<Vector3>();
        var oldToNewMap = new Dictionary<int, int>();
        mapToOriginal = new int[originalVertices.Length];

        for (var i = 0; i < originalVertices.Length; i++)
        {
            var found = false;
            for (var j = 0; j < newVertices.Count; j++)
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
                var newIndex = newVertices.Count - 1;
                oldToNewMap[i] = newIndex;
                mapToOriginal[i] = newIndex;
            }
        }
        
        var newTriangles = new int[originalMesh.triangles.Length];
        for (var i = 0; i < originalMesh.triangles.Length; i++)
        {
            var oldIndex = originalMesh.triangles[i];
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