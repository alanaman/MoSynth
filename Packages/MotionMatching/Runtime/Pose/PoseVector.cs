using System.Diagnostics.Contracts;
using UnityEngine;
using Unity.Mathematics;
using Unity.Mathematics.Geometry;

namespace MotionMatching
{
/// <summary>
/// Stores full pose representation for one pose
/// </summary>
public struct PoseVector
{
    // The first element is the SimulationBone (added artificially), and the rest are the bones of the original skeleton
    public Vector3[] JointLocalPositions;
    public Quaternion[] JointLocalRotations;
    public Vector3[] JointLocalVelocities; // Computed from World Positions
    public Vector3[] JointLocalAngularVelocities; // Computed from World Rotations
    public bool LeftFootContact; // True if the foot is in contact with the ground, false otherwise
    public bool RightFootContact;

    public PoseVector(int numJoints)
    {
        JointLocalPositions = new Vector3[numJoints];
        JointLocalRotations = new Quaternion[numJoints];
        JointLocalVelocities = new Vector3[numJoints];
        JointLocalAngularVelocities = new Vector3[numJoints];
        LeftFootContact = false;
        RightFootContact = false;
    }

    public PoseVector(PoseVector other)
    {
        JointLocalPositions = (Vector3[])other.JointLocalPositions.Clone();
        JointLocalRotations = (Quaternion[])other.JointLocalRotations.Clone();
        JointLocalVelocities = (Vector3[])other.JointLocalVelocities.Clone();
        JointLocalAngularVelocities = (Vector3[])other.JointLocalAngularVelocities.Clone();
        LeftFootContact = other.LeftFootContact;
        RightFootContact = other.RightFootContact;
    }
    
    public PoseVector(Vector3[] jointLocalPositions, Quaternion[] jointLocalRotations,
        Vector3[] jointLocalVelocities, Vector3[] jointLocalAngularVelocities,
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

    /// <summary>
    /// Returns the rotation of the joint in world space after applying FK using the pose
    /// </summary>
    /// <remarks>World here is the immediate parent transform of the Simulation Bone</remarks>
    public Quaternion GetWorldSpaceRotation(Skeleton skeleton, Skeleton.Joint joint)
    {
        var worldRot = Quaternion.identity;
        while (joint.index != 0) // while not root
        {
            worldRot = JointLocalRotations[joint.index] * worldRot;
            joint = skeleton.GetParent(joint);
        }

        worldRot = JointLocalRotations[0] * worldRot; // root
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
    public Quaternion GetHipSpaceRotation(Skeleton skeleton, Skeleton.Joint joint)
    {
        var rot = Quaternion.identity;
        while (joint.index != 0) // while not root
        {
            rot = JointLocalRotations[joint.index] * rot;
            joint = skeleton.GetParent(joint);
        }
        return rot;
    }

    /// <summary>
    /// Returns the position of the joint in character space after applying FK using the pose
    /// </summary>
    public Vector3 GetHipSpacePosition(Skeleton skeleton, Skeleton.Joint joint)
    {
        var localToCharacter = Matrix4x4.identity;
        while (joint.index != 0) // while not root
        {
            // TODO: move this normalization
            JointLocalRotations[joint.index] = Quaternion.Normalize(JointLocalRotations[joint.index]);
            localToCharacter = Matrix4x4.TRS(
                JointLocalPositions[joint.index], JointLocalRotations[joint.index],
                Vector3.one) * localToCharacter;
            joint = skeleton.GetParent(joint);
        }

        return localToCharacter.MultiplyPoint3x4(Vector3.zero);
    }

    /// <summary>
    /// Returns the linear velocity of the joint in world space after applying FK using the pose
    /// </summary>
    public float3 GetWorldSpaceVelocity(Skeleton skeleton, Skeleton.Joint joint)
    {
        var posAcc = Vector3.zero;
        var linVelAcc = Vector3.zero;
        var angVelAcc = Vector3.zero;

        while (joint.index != 0) // while not root
        {
            var p = JointLocalPositions[joint.index];
            var q = JointLocalRotations[joint.index];
            var v = JointLocalVelocities[joint.index];
            var w = JointLocalAngularVelocities[joint.index];

            var rotatedPosAcc = q * posAcc;
            
            // Spatial velocity transfer formula: V = v + (w x rotated_pos) + q * V_acc
            linVelAcc = v + Vector3.Cross(w, rotatedPosAcc) + q * linVelAcc;
            angVelAcc = w + q * angVelAcc;
            posAcc = p + rotatedPosAcc;

            joint = skeleton.GetParent(joint);
        }

        // Apply root (Simulation Bone) transform
        var rootQ = JointLocalRotations[0];
        var rootV = JointLocalVelocities[0];
        var rootW = JointLocalAngularVelocities[0];

        var rootRotatedPosAcc = rootQ * posAcc;
        linVelAcc = rootV + Vector3.Cross(rootW, rootRotatedPosAcc) + rootQ * linVelAcc;

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

            angVelAcc = w + q * angVelAcc;
            joint = skeleton.GetParent(joint);
        }

        var rootQ = JointLocalRotations[0];
        var rootW = JointLocalAngularVelocities[0];

        return rootW + rootQ * angVelAcc;
    }

    /// <summary>
    /// Returns the linear velocity of the joint in character space after applying FK using the pose
    /// </summary>
    public float3 GetCharacterSpaceVelocity(Skeleton skeleton, Skeleton.Joint joint)
    {
        var posAcc = Vector3.zero;
        var linVelAcc = Vector3.zero;
        var angVelAcc = Vector3.zero;

        while (joint.index != 0) // while not root
        {
            var p = JointLocalPositions[joint.index];
            var q = JointLocalRotations[joint.index];
            var v = JointLocalVelocities[joint.index];
            var w = JointLocalAngularVelocities[joint.index];

            var rotatedPosAcc = q * posAcc;

            linVelAcc = v + Vector3.Cross(w, rotatedPosAcc) + q * linVelAcc;
            angVelAcc = w + q * angVelAcc;
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

            angVelAcc = w + q * angVelAcc;
            joint = skeleton.GetParent(joint);
        }

        return angVelAcc;
    }
}
}