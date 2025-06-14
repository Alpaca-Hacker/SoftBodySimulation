using UnityEngine;

namespace SoftBody.Scripts
{
    [System.Serializable]
    public class SoftBodySettings
    {
        [Header("Mesh Generation")]
        public Vector3 size = Vector3.one;
        [Range(2, 20)]
        public int resolution = 4;
    
        [Header("Physics")]
        [Range(0.1f, 10f)]
        public float mass = 1f;
        [Range(0f, 1f)]
        public float damping = 0.01f;
        [Range(0f, 2f)]
        public float gravity = 9.81f;
    
        [Header("Constraints")]
        [Range(0.1f, 1000f)]
        public float stiffness = 100f;
        [Range(1, 10)]
        public int solverIterations = 4;
        [Range(0f, 1f)]
        public float compliance = 0.0001f;
    
        [Header("Interaction")]
        public bool enableCollision = true;
        public LayerMask collisionLayers = -1;
        
        [Header("Debug Options")]
        public bool useCPUFallback = false; 
    }
}