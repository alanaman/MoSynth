using System;
using System.Diagnostics.Contracts;
using UnityEngine;
using Unity.Mathematics;
using Unity.Mathematics.Geometry;

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
    // The first element is the SimulationBone (added artificially), and the rest are the bones of the original skeleton
    public float3[] JointLocalPositions;
    public quaternion[] JointLocalRotations;
    public float3[] JointLocalVelocities; // Computed from World Positions
    public float3[] JointLocalAngularVelocities; // Computed from World Rotations
    public bool LeftFootContact; // True if the foot is in contact with the ground, false otherwise
    public bool RightFootContact;

    public PoseVector(int numJoints)
    {
        JointLocalPositions = new float3[numJoints];
        JointLocalRotations = new quaternion[numJoints];
        JointLocalVelocities = new float3[numJoints];
        JointLocalAngularVelocities = new float3[numJoints];
        LeftFootContact = false;
        RightFootContact = false;
    }

    public PoseVector(PoseVector other)
    {
        JointLocalPositions = (float3[])other.JointLocalPositions.Clone();
        JointLocalRotations = (quaternion[])other.JointLocalRotations.Clone();
        JointLocalVelocities = (float3[])other.JointLocalVelocities.Clone();
        JointLocalAngularVelocities = (float3[])other.JointLocalAngularVelocities.Clone();
        LeftFootContact = other.LeftFootContact;
        RightFootContact = other.RightFootContact;
    }
    
    public PoseVector(float3[] jointLocalPositions, quaternion[] jointLocalRotations,
        float3[] jointLocalVelocities, float3[] jointLocalAngularVelocities,
        bool leftFootContact, bool rightFootContact)
    {
        JointLocalPositions = jointLocalPositions;
        JointLocalRotations = jointLocalRotations;
        JointLocalVelocities = jointLocalVelocities;
        JointLocalAngularVelocities = jointLocalAngularVelocities;
        LeftFootContact = leftFootContact;
        RightFootContact = rightFootContact;
    }

    public void CopyFrom(PoseVector other)
    {
        for (var i = 0; i < JointLocalPositions.Length; i++)
        {
            JointLocalPositions[i] = other.JointLocalPositions[i];
            JointLocalRotations[i] = other.JointLocalRotations[i];
            JointLocalVelocities[i] = other.JointLocalVelocities[i];
            JointLocalAngularVelocities[i] = other.JointLocalAngularVelocities[i];
        }
        LeftFootContact = other.LeftFootContact;
        RightFootContact = other.RightFootContact;
    }
}
}