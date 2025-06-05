using UnityEngine;

namespace SoftBody.Scripts.Models
{
    public class SoftBodyParticle
    {
        public int Id; 
        public float Mass;
        public float InverseMass;
        public Vector3 Position;
        public Vector3 PredictedPosition;
        public Vector3 Velocity;
        public Vector3 ExternalForceAccumulator; 

        public SoftBodyParticle(Vector3 initialPosition, float mass, int id)
        {
            Id = id;
            Mass = mass;
            InverseMass = (mass <= 0.0001f) ? 0f : 1f / mass; // Handle static/infinite mass particles
            Position = initialPosition;
            PredictedPosition = initialPosition;
            Velocity = Vector3.zero;
            ExternalForceAccumulator = Vector3.zero;
        }

        public void AddForce(Vector3 force)
        {
            ExternalForceAccumulator += force;
        }

        public void ClearForces()
        {
            ExternalForceAccumulator = Vector3.zero;
        }
    }
}