using SoftBody.Scripts.Models;
using UnityEngine;

namespace SoftBody.Scripts.Constraints
{
    public class CPUDistanceConstraint
    {
        public SoftBodyParticleCPU ParticleCPUA;
        public SoftBodyParticleCPU ParticleCPUB;
        public float RestLength;
        public float Compliance; // Alpha in XPBD

        private float _lambda; // Lagrange multiplier for this constraint
        private float _maxLambdaChange;
        private readonly bool _isDebugConstraint;

        public CPUDistanceConstraint(SoftBodyParticleCPU particleCPUA, SoftBodyParticleCPU particleCPUB, float restLength, float compliance, float maxLambdaChange, bool isDebugConstraint = false)
        {
            ParticleCPUA = particleCPUA;
            ParticleCPUB = particleCPUB;
            RestLength = restLength;
            Compliance = compliance;
            _lambda = 0f; // Initialize lambda
            _maxLambdaChange = maxLambdaChange;
            _isDebugConstraint = isDebugConstraint;
            if (isDebugConstraint)
            {
                Debug.Log($"DistanceConstraint created between ParticleA({particleCPUA.Id}) and ParticleB({particleCPUB.Id}) with RestLength: {restLength}, Compliance: {compliance}");
            }
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
            if (ParticleCPUA.InverseMass < 0.00001f && ParticleCPUB.InverseMass < 0.00001f) return; // Both particles are static

            if (_isDebugConstraint)
            {
                Debug.Log("---Solving DistanceConstraint between ParticleA(" + ParticleCPUA.Id + ") and ParticleB(" +
                          ParticleCPUB.Id + ")---");
            }

            var direction = ParticleCPUB.PredictedPosition - ParticleCPUA.PredictedPosition;
            var currentLength = direction.magnitude;
            
            if (_isDebugConstraint)
            {
                Debug.Log($"Current Length: {currentLength}, Rest Length: {RestLength}");
            }

            if (currentLength < 0.00001f) return; // Avoid division by zero

            var gradient = direction / currentLength; // Normalized direction

            // Constraint function C = currentLength - RestLength
            var C = currentLength - RestLength;
           
            if (_isDebugConstraint)
            {
                Debug.Log("DistanceConstraint C: " + C);
            }
            

            // XPBD specific: Calculate alpha_tilde (scaled compliance)
            var alphaTilde = Compliance / (deltaTime * deltaTime);
            if (_isDebugConstraint)
            {
                Debug.Log("DistanceConstraint alphaTilde: " + alphaTilde);
            }

            // Calculate deltaLambda
            // The denominator term involves the sum of w_i * |gradient_i|^2. For distance constraint, gradient magnitude is 1.
            // So it's w1*1 + w2*1 = P1.InverseMass + P2.InverseMass
            var denominator = ParticleCPUA.InverseMass + ParticleCPUB.InverseMass + alphaTilde;
            
            if (_isDebugConstraint)
            {
                Debug.Log("DistanceConstraint denominator: " + denominator);
            }
            
            if (Mathf.Abs(denominator) < 0.00001f) return;

            var deltaLambda_raw = (-C - alphaTilde * _lambda) / denominator;
          
            var final_deltaLambda = Mathf.Clamp(deltaLambda_raw, -_maxLambdaChange, _maxLambdaChange);
            _lambda += final_deltaLambda;
            
            if (_isDebugConstraint)
            {
                Debug.Log($"DistanceConstraint deltaLambda_raw: {deltaLambda_raw}, final_deltaLambda: {final_deltaLambda}, _lambda: {_lambda}");
            }

            var dP1 = -ParticleCPUA.InverseMass * final_deltaLambda * gradient;
            var dP2 = ParticleCPUB.InverseMass * final_deltaLambda * gradient;
            
            if (_isDebugConstraint)
            {
                Debug.Log($"dP1 mag: {dP1.magnitude}, dP2 mag: {dP2.magnitude}");
            }

            // Apply corrections to predicted positions
            ParticleCPUA.PredictedPosition += dP1;
            ParticleCPUB.PredictedPosition += dP2;
        }
    }
}