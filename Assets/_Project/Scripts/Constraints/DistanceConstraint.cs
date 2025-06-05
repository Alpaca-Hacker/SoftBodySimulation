using SoftBody.Scripts.Models;
using UnityEngine;

namespace SoftBody.Scripts.Constraints
{
    public class DistanceConstraint
    {
        public SoftBodyParticle ParticleA;
        public SoftBodyParticle ParticleB;
        public float RestLength;
        public float Compliance; // Alpha in XPBD

        private float _lambda; // Lagrange multiplier for this constraint

        public DistanceConstraint(SoftBodyParticle particleA, SoftBodyParticle particleB, float restLength, float compliance)
        {
            ParticleA = particleA;
            ParticleB = particleB;
            RestLength = restLength;
            Compliance = compliance;
            _lambda = 0f; // Initialize lambda
        }

        public void InitializeForSubstep()
        {
            // For some XPBD formulations, lambda might be reset or adjusted per sub-step or physics step.
            // For this basic example, we accumulate lambda across iterations within a single FixedUpdate's solve phase.
            // If you reset it here, it becomes more like PBD unless you carefully manage the XPBD formulation.
            // For true XPBD time-step independent stiffness, lambda accumulates corrections.
            // Let's keep it simple and accumulate for now.
        }
        
        public void ResetLambda()
        {
            _lambda = 0f;
        }

        public void Solve(float deltaTime)
        {
            if (ParticleA.InverseMass < 0.00001f && ParticleB.InverseMass < 0.00001f) return; // Both particles are static

            var direction = ParticleB.PredictedPosition - ParticleA.PredictedPosition;
            var currentLength = direction.magnitude;

            if (currentLength < 0.00001f) return; // Avoid division by zero

            var gradient = direction / currentLength; // Normalized direction

            // Constraint function C = currentLength - RestLength
            var C = currentLength - RestLength;

            // XPBD specific: Calculate alpha_tilde (scaled compliance)
            var alphaTilde = Compliance / (deltaTime * deltaTime);

            // Calculate deltaLambda
            // The denominator term involves the sum of w_i * |gradient_i|^2. For distance constraint, gradient magnitude is 1.
            // So it's w1*1 + w2*1 = P1.InverseMass + P2.InverseMass
            var denominator = ParticleA.InverseMass + ParticleB.InverseMass + alphaTilde;
            if (Mathf.Abs(denominator) < 0.00001f) return;

            var deltaLambda = (-C - alphaTilde * _lambda) / denominator;

            _lambda += deltaLambda; // Accumulate lambda

            // Calculate position corrections (deltaX)
            var dP1 = -ParticleA.InverseMass * deltaLambda * gradient;
            var dP2 = ParticleB.InverseMass * deltaLambda * gradient;

            // Apply corrections to predicted positions
            ParticleA.PredictedPosition += dP1;
            ParticleB.PredictedPosition += dP2;
        }
    }
}