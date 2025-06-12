namespace SoftBody.Scripts.Models
{
    public struct DistanceConstraint
    {
        public uint p1, p2; // Use uint to exactly match the shader
        public float restLength;
        private float _padding; // Pad to 16 bytes (2*4 + 4 + 4 = 16)
    }
}