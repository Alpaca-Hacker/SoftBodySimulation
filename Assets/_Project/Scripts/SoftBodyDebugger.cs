using UnityEditor;
using UnityEngine;

namespace SoftBody.Scripts
{
    public class SoftBodyDebugger : MonoBehaviour
{
    [Header("Debug Visualization")]
    public bool showParticles = false;
    public bool showConstraints = false;
    public bool showForces = false;
    public float particleSize = 0.1f;
    public Color particleColor = Color.red;
    public Color constraintColor = Color.green;
    public Color forceColor = Color.blue;
    
    [Header("Performance Monitoring")]
    public bool showPerformanceStats = true;
    
    private SoftBodySimulator softBody;
    private float frameTime;
    private int frameCount;
    private Vector3[] currentParticlePositions;
    
    private void Start()
    {
        softBody = GetComponent<SoftBodySimulator>();
    }
    
    private void Update()
    {
        if (showPerformanceStats)
        {
            frameTime += Time.deltaTime;
            frameCount++;
        }
        
        // Update particle positions for visualization
        if (softBody && (showParticles || showConstraints))
        {
            UpdateParticlePositions();
        }
    }
    
    private void UpdateParticlePositions()
    {
        // Get current mesh vertices (which represent particle positions)
        var mesh = softBody.GetComponent<MeshFilter>().mesh;
        if (mesh != null)
        {
            var vertices = mesh.vertices;
            currentParticlePositions = new Vector3[vertices.Length];
            
            // Convert from local to world space
            for (var i = 0; i < vertices.Length; i++)
            {
                currentParticlePositions[i] = transform.TransformPoint(vertices[i]);
            }
        }
    }
    
    private void OnDrawGizmos()
    {
        if (softBody == null || currentParticlePositions == null) return;
        
        if (showParticles)
        {
            Gizmos.color = particleColor;
            foreach (var pos in currentParticlePositions)
            {
                Gizmos.DrawWireSphere(pos, particleSize);
            }
        }
        
        if (showConstraints && currentParticlePositions.Length > 0)
        {
            Gizmos.color = constraintColor;
            var res = Mathf.RoundToInt(Mathf.Pow(currentParticlePositions.Length, 1f/3f));
            
            // Draw constraint lines (simplified grid connections)
            for (var x = 0; x < res; x++)
            {
                for (var y = 0; y < res; y++)
                {
                    for (var z = 0; z < res; z++)
                    {
                        var index = x * res * res + y * res + z;
                        if (index >= currentParticlePositions.Length) continue;
                        
                        var pos = currentParticlePositions[index];
                        
                        // Draw connections to adjacent particles
                        if (x < res - 1)
                        {
                            var nextIndex = (x + 1) * res * res + y * res + z;
                            if (nextIndex < currentParticlePositions.Length)
                                Gizmos.DrawLine(pos, currentParticlePositions[nextIndex]);
                        }
                        if (y < res - 1)
                        {
                            var nextIndex = x * res * res + (y + 1) * res + z;
                            if (nextIndex < currentParticlePositions.Length)
                                Gizmos.DrawLine(pos, currentParticlePositions[nextIndex]);
                        }
                        if (z < res - 1)
                        {
                            var nextIndex = x * res * res + y * res + (z + 1);
                            if (nextIndex < currentParticlePositions.Length)
                                Gizmos.DrawLine(pos, currentParticlePositions[nextIndex]);
                        }
                    }
                }
            }
        }
    }
    
    private void OnGUI()
    {
        if (!showPerformanceStats) return;
        
        GUILayout.BeginArea(new Rect(10, 10, 200, 150));
        GUILayout.Label("Soft Body Debug Info", new GUIStyle(GUI.skin.label) { fontStyle = FontStyle.Bold });
        
        if (frameCount > 0)
        {
            var avgFrameTime = frameTime / frameCount;
            var fps = 1f / avgFrameTime;
            GUILayout.Label($"FPS: {fps:F1}");
            GUILayout.Label($"Frame Time: {avgFrameTime * 1000:F2}ms");
        }
        
        if (softBody != null && currentParticlePositions != null)
        {
            GUILayout.Label($"Particles: {currentParticlePositions.Length}");
            GUILayout.Label("Simulation Active");
        }
        
        GUILayout.EndArea();
        
        // Reset counters periodically
        if (frameCount > 60)
        {
            frameTime = 0f;
            frameCount = 0;
        }
    }
}
}