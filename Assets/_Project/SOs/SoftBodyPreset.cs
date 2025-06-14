using UnityEngine;

namespace SoftBody.Scripts
{
    [CreateAssetMenu(fileName = "SoftBodyPreset", menuName = "Soft Body/Preset")]
    public class SoftBodyPreset : ScriptableObject
    {
        [Header("Preset Information")]
        public string presetName = "Custom Preset";
        [TextArea(3, 5)]
        public string description = "Custom soft body configuration";
    
        [Header("Settings")]
        public SoftBodySettings settings;
    
        public void ApplyToSoftBody(SoftBodySimulator softBody)
        {
            // Use reflection to copy settings
            var fields = typeof(SoftBodySettings).GetFields();
            foreach (var field in fields)
            {
                field.SetValue(softBody.GetType().GetField("settings", 
                        System.Reflection.BindingFlags.NonPublic | 
                        System.Reflection.BindingFlags.Instance).GetValue(softBody), 
                    field.GetValue(settings));
            }
        }
    }
}