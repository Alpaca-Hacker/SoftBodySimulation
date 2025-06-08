using SoftBody.Scripts.Models;
using UnityEngine;

public class BendingConstraint
{
    public SoftBodyParticleCPU ParticleCPUA;
    public SoftBodyParticleCPU ParticleCPUB;
    public SoftBodyParticleCPU ParticleCPUC;
    public SoftBodyParticleCPU ParticleCPUD;
    
    public float RestAngle; // Radians
    public float Compliance; 
    
    private float _lambda; 
    private readonly bool _isDebugConstraint = false; // Flag for debugging specific constraints
    private float _maxLambdaChange;

    public BendingConstraint(SoftBodyParticleCPU particleCPUA, SoftBodyParticleCPU particleCPUB, 
        SoftBodyParticleCPU particleCPUC, SoftBodyParticleCPU particleCPUD, float restAngle, float compliance, bool isDebugConstraint, float maxLambdaChange)
    {
        ParticleCPUA = particleCPUA;
        ParticleCPUB = particleCPUB;
        ParticleCPUC = particleCPUC;
        ParticleCPUD = particleCPUD;
        RestAngle = restAngle;
        Compliance = compliance;
        _isDebugConstraint = isDebugConstraint;
        _lambda = 0f; 
        _maxLambdaChange = maxLambdaChange; // Set the maximum change in lambda per solve step
        
    }
    
    public void ResetLambda()
    {
        _lambda = 0f; 
    }

    // In BendingConstraint.cs

public void Solve(float deltaTime)
{
    if (ParticleCPUA.InverseMass < 0.00001f && ParticleCPUB.InverseMass < 0.00001f &&
        ParticleCPUC.InverseMass < 0.00001f && ParticleCPUD.InverseMass < 0.00001f)
    {
        return;
    }

    var pA_pred = ParticleCPUA.PredictedPosition;
    var pB_pred = ParticleCPUB.PredictedPosition;
    var pC_pred = ParticleCPUC.PredictedPosition;
    var pD_pred = ParticleCPUD.PredictedPosition;

    var e0 = pB_pred - pA_pred;
    var e1 = pC_pred - pA_pred;
    var e2 = pD_pred - pA_pred;

    var n1_vec = Vector3.Cross(e0, e1);
    var n2_vec = Vector3.Cross(e2, e0);

    var l_n1_sq = n1_vec.sqrMagnitude;
    var l_n2_sq = n2_vec.sqrMagnitude;

    const float geomEpsilon = 1e-9f;
    if (l_n1_sq < geomEpsilon || l_n2_sq < geomEpsilon)
    {
        // if (_isDebugConstraint) Debug.LogWarning("BendingConstraint: Degenerate triangle, skipping solve.");
        return;
    }

    var l_n1 = Mathf.Sqrt(l_n1_sq);
    var l_n2 = Mathf.Sqrt(l_n2_sq);

    var n1_norm = n1_vec / l_n1;
    var n2_norm = n2_vec / l_n2;

    var cosTheta = Mathf.Clamp(Vector3.Dot(n1_norm, n2_norm), -1.0f, 1.0f);
    var currentAngle = Mathf.Acos(cosTheta);
    var C = currentAngle - RestAngle;

    // Initialize gradients to zero - they will be calculated if sinTheta is safe
    var grad_pA = Vector3.zero;
    var grad_pB = Vector3.zero;
    var grad_pC = Vector3.zero;
    var grad_pD = Vector3.zero;
    var sum_grad_sq_w = 0f;

    var sinTheta = Mathf.Sin(currentAngle);
    var final_deltaLambda_to_accumulate = 0f;

    var current_alpha_tilde = Compliance / (deltaTime * deltaTime); // Base alpha_tilde

    var stabilityEpsilonForSinTheta = 0.01f;
    var dangerousEpsilonForSinTheta = 1e-5f; // Much smaller, for definite zeroing

    if (Mathf.Abs(sinTheta) < dangerousEpsilonForSinTheta) // Very dangerous, definitely skip
    {
        if (_isDebugConstraint) Debug.LogWarning($"BendingConstraint: sinTheta ({sinTheta:F7}) is EXTREMELY small. Applying NO correction.");
        // final_deltaLambda_to_accumulate remains 0
    }
    else if (Mathf.Abs(sinTheta) < stabilityEpsilonForSinTheta) // Small, but not zero - soften
    {
        if (_isDebugConstraint)
            Debug.LogWarning($"BendingConstraint: sinTheta ({sinTheta:F7}) is small. Scaling effective_alpha_tilde.");
        {
            current_alpha_tilde *= 100.0f;
        }
        // Make constraint much softer, gradients will be calculated normally
        // but deltaLambda will be smaller due to larger alpha_tilde in denominator.
        // Proceed to calculate gradients and deltaLambda with this scaled current_alpha_tilde
        var invSinTheta = 1.0f / sinTheta; // Still might be large
        if (float.IsInfinity(invSinTheta) || float.IsNaN(invSinTheta))
        {
            /* final_deltaLambda_to_accumulate remains 0 */
        }
        else
        {
            var term_gu_common = (Vector3.Cross(n1_norm, e0) * cosTheta - Vector3.Cross(n2_norm, e0)) * invSinTheta;
            var term_gv_common = (Vector3.Cross(e0, n2_norm) * cosTheta - Vector3.Cross(e0, n1_norm)) * invSinTheta;

            // Gradients are now calculated because sinTheta was safe
            grad_pC = Vector3.Cross(e0, term_gu_common / l_n1);
            grad_pD = Vector3.Cross(term_gv_common / l_n2, e0);
            grad_pB = Vector3.Cross(e1, term_gu_common / l_n1) + Vector3.Cross(term_gv_common / l_n2, e2);
            grad_pA = -grad_pB - grad_pC - grad_pD;

            sum_grad_sq_w = ParticleCPUA.InverseMass * grad_pA.sqrMagnitude +
                            ParticleCPUB.InverseMass * grad_pB.sqrMagnitude +
                            ParticleCPUC.InverseMass * grad_pC.sqrMagnitude +
                            ParticleCPUD.InverseMass * grad_pD.sqrMagnitude;

            if (sum_grad_sq_w + current_alpha_tilde >= geomEpsilon)
            {
                var deltaLambda_raw = (-C - current_alpha_tilde * _lambda) / (sum_grad_sq_w + current_alpha_tilde);
                final_deltaLambda_to_accumulate = Mathf.Clamp(deltaLambda_raw, -_maxLambdaChange, _maxLambdaChange);
            }
        }

        _lambda += final_deltaLambda_to_accumulate;
        // Optionally clamp accumulated _lambda:
        // float maxTotalLambda = 0.01f; // EXPERIMENT
        // _lambda = Mathf.Clamp(_lambda, -maxTotalLambda, maxTotalLambda);
    }

    if (_isDebugConstraint)
    {
        var alpha_tilde_for_log = Compliance / (deltaTime * deltaTime);
        Debug.Log($"--- Bending Constraint Debug (P0:{ParticleCPUA.Id}, P1:{ParticleCPUB.Id}, P2:{ParticleCPUC.Id}, P3:{ParticleCPUD.Id}) ---");
        Debug.Log($"C: {C:F7}, Current Angle: {currentAngle * Mathf.Rad2Deg:F2} deg, Rest: {RestAngle * Mathf.Rad2Deg:F2} deg");
        Debug.Log($"sinTheta: {sinTheta:F7}, invSinTheta (if used): {(Mathf.Abs(sinTheta) < stabilityEpsilonForSinTheta ? "N/A (sinTheta too small)" : (1.0f/sinTheta).ToString("F7"))}");
        Debug.Log($"grad_pA: {grad_pA.magnitude:F7}, grad_pB: {grad_pB.magnitude:F7}, grad_pC: {grad_pC.magnitude:F7}, grad_pD: {grad_pD.magnitude:F7}");
        Debug.Log($"sum_w_grad_sq (if used): {sum_grad_sq_w:F7}, alpha_tilde (base): {alpha_tilde_for_log:F7}");
        // Note: deltaLambda_raw isn't well-defined if we skipped due to sinTheta, so not logging it directly unless calculated
        Debug.Log($"deltaLambda (applied to lambda): {final_deltaLambda_to_accumulate:F7}, _lambda (new total): {_lambda:F7}");
    }

    // Apply corrections only if deltaLambda to apply is significant enough AND gradients were valid (i.e. not zeroed from start)
    if (Mathf.Abs(final_deltaLambda_to_accumulate) > geomEpsilon)
    {
        // The gradients grad_pA etc. will be non-zero only if we went through the 'else' block of the sinTheta check.
        // If sinTheta was too small, gradients remain Vector3.zero, so no correction is applied here, which is correct.
        ParticleCPUA.PredictedPosition += ParticleCPUA.InverseMass * final_deltaLambda_to_accumulate * grad_pA;
        ParticleCPUB.PredictedPosition += ParticleCPUB.InverseMass * final_deltaLambda_to_accumulate * grad_pB;
        ParticleCPUC.PredictedPosition += ParticleCPUC.InverseMass * final_deltaLambda_to_accumulate * grad_pC;
        ParticleCPUD.PredictedPosition += ParticleCPUD.InverseMass * final_deltaLambda_to_accumulate * grad_pD;
    }
}


    public static float CalculateRestAngle(Vector3 particleA, Vector3 particleB, Vector3 particleC, Vector3 particleD)
    {
        var e0 = particleB - particleA;
        var e1 = particleC - particleA;
        var e2 = particleD - particleA;

        var n1_vec = Vector3.Cross(e0, e1);
        var n2_vec = Vector3.Cross(e2, e0);

        var l_n1_sq = n1_vec.sqrMagnitude;
        var l_n2_sq = n2_vec.sqrMagnitude;

        if (l_n1_sq < 1e-9f || l_n2_sq < 1e-9f) return 0f; // Or Mathf.PI for flat

        var n1_norm = n1_vec.normalized;
        var n2_norm = n2_vec.normalized;

        var cosTheta = Mathf.Clamp(Vector3.Dot(n1_norm, n2_norm), -1.0f, 1.0f);
        return Mathf.Acos(cosTheta);
    }
 }
