using System.Diagnostics.Contracts;
using UnityEngine;
using Unity.Mathematics;

namespace MotionMatching
{
/// <summary>
/// Stores full pose representation for one pose
/// </summary>
public struct PoseVector
{
    // The first element is the SimulationBone (added artificially), and the rest are the bones of the original skeleton
    public float3[] JointLocalPositions;
    public quaternion[] JointLocalRotations;
    public float3[] JointLocalVelocities; // Computed from World Positions
    public float3[] JointLocalAngularVelocities; // Computed from World Rotations
    public bool LeftFootContact; // True if the foot is in contact with the ground, false otherwise
    public bool RightFootContact;

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

    /// <summary>
    /// Returns the rotation of the joint in world space after applying FK using the pose
    /// </summary>
    /// <remarks>World here is the immediate parent transform of the Simulation Bone</remarks>
    public quaternion GetWorldSpaceRotation(Skeleton skeleton, Skeleton.Joint joint)
    {
        var worldRot = quaternion.identity;
        while (joint.index != 0) // while not root
        {
            worldRot = math.mul(JointLocalRotations[joint.index], worldRot);
            joint = skeleton.GetParent(joint);
        }

        worldRot = math.mul(JointLocalRotations[0], worldRot); // root
        return worldRot;
    }

    /// <summary>
    /// Returns the position of the joint in world space after applying FK using the pose
    /// </summary>
    /// <remarks>World here is the immediate parent transform of the Simulation Bone</remarks>
    public float3 GetWorldSpacePosition(Skeleton skeleton, Skeleton.Joint joint)
    {
        var localToWorld = Matrix4x4.identity;
        while (joint.index != 0) // while not root
        {
            localToWorld = Matrix4x4.TRS(JointLocalPositions[joint.index], JointLocalRotations[joint.index],
                new float3(1.0f, 1.0f, 1.0f)) * localToWorld;
            joint = skeleton.GetParent(joint);
        }

        localToWorld = Matrix4x4.TRS(JointLocalPositions[0], JointLocalRotations[0], new float3(1.0f, 1.0f, 1.0f)) *
                       localToWorld; // root
        return localToWorld.MultiplyPoint3x4(Vector3.zero);
    }

    /// <summary>
    /// Returns the rotation of the joint in character space after applying FK using the pose
    /// </summary>
    public quaternion GetCharacterSpaceRotation(Skeleton skeleton, Skeleton.Joint joint)
    {
        var rot = quaternion.identity;
        while (joint.index != 0) // while not root
        {
            rot = math.mul(JointLocalRotations[joint.index], rot);
            joint = skeleton.GetParent(joint);
        }

        return rot;
    }

    /// <summary>
    /// Returns the position of the joint in character space after applying FK using the pose
    /// </summary>
    public float3 GetCharacterSpacePosition(Skeleton skeleton, Skeleton.Joint joint)
    {
        var localToCharacter = Matrix4x4.identity;
        while (joint.index != 0) // while not root
        {
            localToCharacter = Matrix4x4.TRS(JointLocalPositions[joint.index], JointLocalRotations[joint.index],
                new float3(1.0f, 1.0f, 1.0f)) * localToCharacter;
            joint = skeleton.GetParent(joint);
        }

        return localToCharacter.MultiplyPoint3x4(Vector3.zero);
    }

    /// <summary>
    /// Returns the linear velocity of the joint in world space after applying FK using the pose
    /// </summary>
    public float3 GetWorldSpaceVelocity(Skeleton skeleton, Skeleton.Joint joint)
    {
        var posAcc = float3.zero;
        var linVelAcc = float3.zero;
        var angVelAcc = float3.zero;

        while (joint.index != 0) // while not root
        {
            var p = JointLocalPositions[joint.index];
            var q = JointLocalRotations[joint.index];
            var v = JointLocalVelocities[joint.index];
            var w = JointLocalAngularVelocities[joint.index];

            var rotatedPosAcc = math.mul(q, posAcc);

            // Spatial velocity transfer formula: V = v + (w x rotated_pos) + q * V_acc
            linVelAcc = v + math.cross(w, rotatedPosAcc) + math.mul(q, linVelAcc);
            angVelAcc = w + math.mul(q, angVelAcc);
            posAcc = p + rotatedPosAcc;

            joint = skeleton.GetParent(joint);
        }

        // Apply root (Simulation Bone) transform
        var rootQ = JointLocalRotations[0];
        var rootV = JointLocalVelocities[0];
        var rootW = JointLocalAngularVelocities[0];

        var rootRotatedPosAcc = math.mul(rootQ, posAcc);
        linVelAcc = rootV + math.cross(rootW, rootRotatedPosAcc) + math.mul(rootQ, linVelAcc);

        return linVelAcc;
    }

    /// <summary>
    /// Returns the angular velocity of the joint in world space after applying FK using the pose
    /// </summary>
    public float3 GetWorldSpaceAngularVelocity(Skeleton skeleton, Skeleton.Joint joint)
    {
        var angVelAcc = float3.zero;

        while (joint.index != 0)
        {
            var q = JointLocalRotations[joint.index];
            var w = JointLocalAngularVelocities[joint.index];

            angVelAcc = w + math.mul(q, angVelAcc);
            joint = skeleton.GetParent(joint);
        }

        var rootQ = JointLocalRotations[0];
        var rootW = JointLocalAngularVelocities[0];

        return rootW + math.mul(rootQ, angVelAcc);
    }

    /// <summary>
    /// Returns the linear velocity of the joint in character space after applying FK using the pose
    /// </summary>
    public float3 GetCharacterSpaceVelocity(Skeleton skeleton, Skeleton.Joint joint)
    {
        var posAcc = float3.zero;
        var linVelAcc = float3.zero;
        var angVelAcc = float3.zero;

        while (joint.index != 0) // while not root
        {
            var p = JointLocalPositions[joint.index];
            var q = JointLocalRotations[joint.index];
            var v = JointLocalVelocities[joint.index];
            var w = JointLocalAngularVelocities[joint.index];

            var rotatedPosAcc = math.mul(q, posAcc);

            linVelAcc = v + math.cross(w, rotatedPosAcc) + math.mul(q, linVelAcc);
            angVelAcc = w + math.mul(q, angVelAcc);
            posAcc = p + rotatedPosAcc;

            joint = skeleton.GetParent(joint);
        }

        return linVelAcc;
    }

    /// <summary>
    /// Returns the angular velocity of the joint in character space after applying FK using the pose
    /// </summary>
    public float3 GetCharacterSpaceAngularVelocity(Skeleton skeleton, Skeleton.Joint joint)
    {
        var angVelAcc = float3.zero;

        while (joint.index != 0)
        {
            var q = JointLocalRotations[joint.index];
            var w = JointLocalAngularVelocities[joint.index];

            angVelAcc = w + math.mul(q, angVelAcc);
            joint = skeleton.GetParent(joint);
        }

        return angVelAcc;
    }
}
}