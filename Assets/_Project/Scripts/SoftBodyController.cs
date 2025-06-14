
using UnityEngine;
using UnityEngine.Events;

namespace SoftBody.Scripts
{
    
    [System.Serializable]
    public class SoftBodyEvent : UnityEvent<Vector3, float>
    {
    }

    public class SoftBodyController : MonoBehaviour
    {
        [Header("Control Settings")] public bool enableKeyboardControl = true;
        public bool enableMouseControl = true;
        public bool enablePhysicsInteraction = true;

        [Header("Presets")] public SoftBodyPreset jellyCubePreset;
        public SoftBodyPreset firmCubePreset;
        public SoftBodyPreset bouncyCubePreset;

        [Header("Events")] public SoftBodyEvent onForceApplied;
        public UnityEvent onDeformationDetected;

        private SoftBodySimulator softBody;
        private SoftBodyAnimator animator;
        private Vector3 initialPosition;
        private float deformationThreshold = 0.5f;

        private void Start()
        {
            softBody = GetComponent<SoftBodySimulator>();
            animator = GetComponent<SoftBodyAnimator>();
            initialPosition = transform.position;

            // Apply default preset if available
            if (jellyCubePreset != null)
            {
                ApplyPreset(jellyCubePreset);
            }
        }

        private void Update()
        {
            HandleInput();
            CheckDeformation();
        }

        private void HandleInput()
        {
            if (!enableKeyboardControl) return;

            // Preset switching
            if (Input.GetKeyDown(KeyCode.Alpha1) && jellyCubePreset != null)
                ApplyPreset(jellyCubePreset);
            if (Input.GetKeyDown(KeyCode.Alpha2) && firmCubePreset != null)
                ApplyPreset(firmCubePreset);
            if (Input.GetKeyDown(KeyCode.Alpha3) && bouncyCubePreset != null)
                ApplyPreset(bouncyCubePreset);

            // Animation triggers
            if (Input.GetKeyDown(KeyCode.Return) && animator != null)
                animator.PlayForceAnimation();
            if (Input.GetKeyDown(KeyCode.Q) && animator != null)
                animator.Squeeze();

            // Reset
            if (Input.GetKeyDown(KeyCode.R))
                ResetSoftBody();
        }

        private void CheckDeformation()
        {
            var distanceFromInitial = Vector3.Distance(transform.position, initialPosition);
            if (distanceFromInitial > deformationThreshold)
            {
                onDeformationDetected?.Invoke();
            }
        }

        public void ApplyPreset(SoftBodyPreset preset)
        {
            if (preset != null && softBody != null)
            {
                preset.ApplyToSoftBody(softBody);
                Debug.Log($"Applied preset: {preset.presetName}");
            }
        }

        public void ApplyForce(Vector3 force)
        {
            if (softBody != null)
            {
                softBody.AddForce(force, transform.position, 2f);
                onForceApplied?.Invoke(force, force.magnitude);
            }
        }

        public void ResetSoftBody()
        {
            transform.position = initialPosition;
            // Force regeneration
            if (softBody != null)
            {
                softBody.SendMessage("OnValidate", SendMessageOptions.DontRequireReceiver);
            }
        }

        private void OnTriggerEnter(Collider other)
        {
            if (!enablePhysicsInteraction) return;

            // Apply force based on collision
            var forceDirection = (transform.position - other.transform.position).normalized;
            var forceMagnitude = 30f;

            ApplyForce(forceDirection * forceMagnitude);
        }
    }
}