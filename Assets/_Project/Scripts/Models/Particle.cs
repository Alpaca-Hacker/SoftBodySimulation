using UnityEngine;

namespace SoftBody.Scripts.Models
{
    public struct Particle
    {
        public Vector4 position;
        public Vector4 predictedPosition;
        public Vector4 old_position;
    }
}