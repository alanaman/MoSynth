using System;
using Unity.Mathematics;

namespace MotionMatching
{
/// <summary>
/// Stores full pose representation for one pose.
/// A <see cref="Skeleton"/> corresponding to this pose will be contained in
/// the <see cref="PoseSet"/> that this <see cref="PoseVector"/> belongs to.
/// </summary>
[Serializable]
public struct PoseVector
{
    // TODO: separate root motion into its own field
    // The first element is the SimulationBone (added artificially), and the rest are the bones of the original skeleton
    public float3[] jointLocalPositions;
    public quaternion[] jointLocalRotations;
    public float3[] jointLocalVelocities; // Computed from World Positions
    public float3[] jointLocalAngularVelocities; // Computed from World Rotations
    public bool leftFootContact; // True if the foot is in contact with the ground, false otherwise
    public bool rightFootContact;

    public PoseVector(int numJoints)
    {
        jointLocalPositions = new float3[numJoints];
        jointLocalRotations = new quaternion[numJoints];
        jointLocalVelocities = new float3[numJoints];
        jointLocalAngularVelocities = new float3[numJoints];
        leftFootContact = false;
        rightFootContact = false;
    }

    public PoseVector(PoseVector other)
    {
        jointLocalPositions = (float3[])other.jointLocalPositions.Clone();
        jointLocalRotations = (quaternion[])other.jointLocalRotations.Clone();
        jointLocalVelocities = (float3[])other.jointLocalVelocities.Clone();
        jointLocalAngularVelocities = (float3[])other.jointLocalAngularVelocities.Clone();
        leftFootContact = other.leftFootContact;
        rightFootContact = other.rightFootContact;
    }
    
    public PoseVector(float3[] jointLocalPositions, quaternion[] jointLocalRotations,
        float3[] jointLocalVelocities, float3[] jointLocalAngularVelocities,
        bool leftFootContact, bool rightFootContact)
    {
        this.jointLocalPositions = jointLocalPositions;
        this.jointLocalRotations = jointLocalRotations;
        this.jointLocalVelocities = jointLocalVelocities;
        this.jointLocalAngularVelocities = jointLocalAngularVelocities;
        this.leftFootContact = leftFootContact;
        this.rightFootContact = rightFootContact;
    }

    public void CopyFrom(PoseVector other)
    {
        for (var i = 0; i < jointLocalPositions.Length; i++)
        {
            jointLocalPositions[i] = other.jointLocalPositions[i];
            jointLocalRotations[i] = other.jointLocalRotations[i];
            jointLocalVelocities[i] = other.jointLocalVelocities[i];
            jointLocalAngularVelocities[i] = other.jointLocalAngularVelocities[i];
        }
        leftFootContact = other.leftFootContact;
        rightFootContact = other.rightFootContact;
    }
}
}