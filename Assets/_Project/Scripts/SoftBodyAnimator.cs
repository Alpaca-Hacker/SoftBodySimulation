using System.Collections;
using UnityEngine;
using UnityEngine.Events;
using Input = UnityEngine.Windows.Input;

namespace SoftBody.Scripts
{
    public class SoftBodyAnimator : MonoBehaviour
    {
        [Header("Animation Settings")] 
        public AnimationCurve forceCurve = AnimationCurve.EaseInOut(0, 0, 1, 1);
        public float animationDuration = 2f;
        public Vector3 forceDirection = Vector3.up;
        public float maxForce = 100f;

        [Header("Pulsing Effect")] public bool enablePulsing = false;
        public float pulseFrequency = 1f;
        public float pulseStrength = 20f;

        private SoftBodySimulator softBody;

        private void Start()
        {
            softBody = GetComponent<SoftBodySimulator>();
            if (softBody == null)
            {
                Debug.LogError("SoftBodyAnimator requires a SoftBodySimulator component!");
                return;
            }

            if (enablePulsing)
            {
                StartCoroutine(PulsingEffect());
            }
        }

        public void PlayForceAnimation()
        {
            StartCoroutine(AnimateForce());
        }

        private IEnumerator AnimateForce()
        {
            float elapsed = 0f;

            while (elapsed < animationDuration)
            {
                elapsed += Time.deltaTime;
                float t = elapsed / animationDuration;
                float forceMultiplier = forceCurve.Evaluate(t);

                Vector3 force = forceDirection.normalized * maxForce * forceMultiplier;
                softBody.AddForce(force, transform.position, 2f);

                yield return null;
            }
        }

        private IEnumerator PulsingEffect()
        {
            while (enablePulsing)
            {
                float pulseForce = Mathf.Sin(Time.time * pulseFrequency * Mathf.PI * 2) * pulseStrength;
                Vector3 force = Vector3.up * pulseForce;
                softBody.AddForce(force, transform.position, 1f);

                yield return null;
            }
        }

        public void Squeeze(float intensity = 1f)
        {
            StartCoroutine(SqueezeEffect(intensity));
        }

        private IEnumerator SqueezeEffect(float intensity)
        {
            Vector3 center = transform.position;
            float duration = 1f;
            float elapsed = 0f;

            while (elapsed < duration)
            {
                elapsed += Time.deltaTime;
                float t = elapsed / duration;
                float squeezeForce = Mathf.Sin(t * Mathf.PI) * intensity * 50f;

                // Apply inward force
                Vector3 inwardForce = (center - transform.position).normalized * squeezeForce;
                softBody.AddForce(inwardForce, center, 3f);

                yield return null;
            }
        }
    }
}
