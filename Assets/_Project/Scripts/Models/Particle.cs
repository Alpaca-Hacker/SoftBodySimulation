using UnityEngine;

namespace SoftBody.Scripts.Models
{
    public struct Particle
    {
        public Vector3 position;
        public Vector3 predictedPosition;
        public Vector3 velocity;
        public float inverseMass;
    }
}