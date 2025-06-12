using UnityEngine;

namespace SoftBody.Scripts.Models
{
    public struct Particle
    {
        public Vector4 position;          // position is in xyz, inverseMass is in w
        public Vector4 predictedPosition; // predictedPosition is in xyz
        public Vector4 velocity;          // velocity is in xyz
    }
}