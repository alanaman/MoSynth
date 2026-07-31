using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{
[Serializable]
public class Skeleton
{
    [field: SerializeField]
    public List<Joint> Joints { get; private set; } = new();

    public void AddJoint(Joint joint)
    {
        Joints.Add(joint);
    }

    public int GetJointIndex(HumanBodyBones type)
    {
        return Find(type).index;
    }
    
    public Joint Find(HumanBodyBones type)
    {
        if(!TryFind(type, out Joint joint))
        {
            throw new Exception($"Joint of type {type} not found in skeleton.");
        }
        return joint;
    }
    
    public bool TryFind(HumanBodyBones type, out Joint joint)
    {
        for (int i = 0; i < Joints.Count; i++)
        {
            if (Joints[i].type == type)
            {
                joint = Joints[i];
                return true;
            }
        }

        joint = new Joint();
        return false;
    }

    public bool TryFind(string jointName, out Joint joint)
    {
        for (int i = 0; i < Joints.Count; i++)
        {
            if (Joints[i].name == jointName)
            {
                joint = Joints[i];
                return true;
            }
        }

        joint = new Joint();
        return false;
    }

    public Joint GetParent(Joint joint)
    {
        return Joints[joint.parentIndex];
    }

    [Serializable]
    public struct Joint : IEquatable<Joint>
    {
        public string name;
        public int index;
        public int parentIndex; // The Root has ParentIndex = -1
        public float3 localOffset;
        public HumanBodyBones type;

        public Joint(string name, int index, int parentIndex, float3 localOffset)
        {
            this.name = name;
            this.index = index;
            this.parentIndex = parentIndex;
            this.localOffset = localOffset;
            type = HumanBodyBones.LastBone;
        }

        public Joint(string name, int index, int parentIndex, float3 localOffset, HumanBodyBones type)
        {
            this.name = name;
            this.index = index;
            this.parentIndex = parentIndex;
            this.localOffset = localOffset;
            this.type = type;
        }

        public bool Equals(Joint other)
        {
            return name == other.name && index == other.index && parentIndex == other.parentIndex &&
                   localOffset.Equals(other.localOffset) && type == other.type;
        }
    }

    /// <summary>
    /// Returns the rotation of the joint in world space after applying FK using the pose
    /// </summary>
    /// <remarks>World here is the immediate parent transform of the Simulation Bone</remarks>
    public quaternion GetWorldSpaceRotation(Joint joint, PoseVector poseVector)
    {
        var worldRot = quaternion.identity;
        while (joint.index != 0) // while not root
        {
            worldRot = math.mul(poseVector.JointLocalRotations[joint.index], worldRot);
            joint = GetParent(joint);
        }

        worldRot = math.mul(poseVector.JointLocalRotations[0], worldRot); // root
        return worldRot;
    }

    /// <summary>
    /// Returns the position of the joint in world space after applying FK using the pose
    /// </summary>
    /// <remarks>World here is the immediate parent transform of the Simulation Bone</remarks>
    public float3 GetWorldSpacePosition(Joint joint, PoseVector poseVector)
    {
        var localToWorld = Matrix4x4.identity;
        while (joint.index != 0) // while not root
        {
            localToWorld = Matrix4x4.TRS(poseVector.JointLocalPositions[joint.index], poseVector.JointLocalRotations[joint.index],
                new float3(1.0f, 1.0f, 1.0f)) * localToWorld;
            joint = GetParent(joint);
        }

        localToWorld = Matrix4x4.TRS(poseVector.JointLocalPositions[0], poseVector.JointLocalRotations[0], new float3(1.0f, 1.0f, 1.0f)) *
                       localToWorld; // root
        return localToWorld.MultiplyPoint3x4(Vector3.zero);
    }

    /// <summary>
    /// Returns the linear velocity of the joint in world space after applying FK using the pose
    /// </summary>
    public float3 GetWorldSpaceVelocity(Joint joint, PoseVector poseVector)
    {
        var posAcc = float3.zero;
        var linVelAcc = float3.zero;
        var angVelAcc = float3.zero;

        while (joint.index != 0) // while not root
        {
            var p = poseVector.JointLocalPositions[joint.index];
            var q = poseVector.JointLocalRotations[joint.index];
            var v = poseVector.JointLocalVelocities[joint.index];
            var w = poseVector.JointLocalAngularVelocities[joint.index];

            var rotatedPosAcc = math.mul(q, posAcc);
            
            // Spatial velocity transfer formula: V = v + (w x rotated_pos) + q * V_acc
            linVelAcc = v + math.cross(w, rotatedPosAcc) + math.mul(q, linVelAcc);
            angVelAcc = w + math.mul(q, angVelAcc);
            posAcc = p + rotatedPosAcc;

            joint = GetParent(joint);
        }

        // Apply root (Simulation Bone) transform
        var rootQ = poseVector.JointLocalRotations[0];
        var rootV = poseVector.JointLocalVelocities[0];
        var rootW = poseVector.JointLocalAngularVelocities[0];

        var rootRotatedPosAcc = math.mul(rootQ, posAcc);
        linVelAcc = rootV + math.cross(rootW, rootRotatedPosAcc) + math.mul(rootQ, linVelAcc);

        return linVelAcc;
    }

    /// <summary>
    /// Returns the angular velocity of the joint in world space after applying FK using the pose
    /// </summary>
    public float3 GetWorldSpaceAngularVelocity(Joint joint, PoseVector poseVector)
    {
        var angVelAcc = float3.zero;

        while (joint.index != 0)
        {
            var q = poseVector.JointLocalRotations[joint.index];
            var w = poseVector.JointLocalAngularVelocities[joint.index];

            angVelAcc = w + math.mul(q, angVelAcc);
            joint = GetParent(joint);
        }

        var rootQ = poseVector.JointLocalRotations[0];
        var rootW = poseVector.JointLocalAngularVelocities[0];

        return rootW + math.mul(rootQ, angVelAcc);
    }

    /// <summary>
    /// Returns the rotation of the joint in the root space (Simulation Bone)
    /// after applying FK.
    /// </summary>
    public quaternion GetRootSpaceRotation(Joint joint, PoseVector poseVector)
    {
        var rot = quaternion.identity;
        while (joint.index != 0) // while not root
        {
            rot = math.mul(poseVector.JointLocalRotations[joint.index], rot);
            joint = GetParent(joint);
        }
        return rot;
    }

    /// <summary>
    /// Returns the position of the joint in the root space (Simulation Bone)
    /// after applying FK.
    /// </summary>
    public float3 GetRootSpacePosition(Joint joint, PoseVector poseVector)
    {
        var localToCharacter = float4x4.identity;
        while (joint.index != 0) // while not root
        {
            var localRot = math.float3x3(poseVector.JointLocalRotations[joint.index]);
            var localTransform = math.float4x4(localRot, poseVector.JointLocalPositions[joint.index]);
            localToCharacter = math.mul(localTransform, localToCharacter);
            joint = GetParent(joint);
        }

        return math.mul(localToCharacter, math.float4(Vector3.zero, 1.0f)).xyz;
    }

    /// <summary>
    /// Returns the linear velocity of the joint in the root space (Simulation Bone)
    /// after applying FK.
    /// </summary>
    public float3 GetRootSpaceVelocity(Joint joint, PoseVector poseVector)
    {
        var posAcc = float3.zero;
        var linVelAcc = float3.zero;
        var angVelAcc = float3.zero;

        while (joint.index != 0) // while not root
        {
            var p = poseVector.JointLocalPositions[joint.index];
            var q = poseVector.JointLocalRotations[joint.index];
            var v = poseVector.JointLocalVelocities[joint.index];
            var w = poseVector.JointLocalAngularVelocities[joint.index];

            var rotatedPosAcc = math.mul(q, posAcc);

            linVelAcc = v + math.cross(w, rotatedPosAcc) + math.mul(q, linVelAcc);
            angVelAcc = w + math.mul(q, angVelAcc);
            posAcc = p + rotatedPosAcc;

            joint = GetParent(joint);
        }

        return linVelAcc;
    }

    /// <summary>
    /// Returns the angular velocity of the joint in the root space (Simulation Bone)
    /// after applying FK.
    /// </summary>
    public float3 GetRootSpaceAngularVelocity(Joint joint, PoseVector poseVector)
    {
        var angVelAcc = float3.zero;

        while (joint.index != 0)
        {
            var q = poseVector.JointLocalRotations[joint.index];
            var w = poseVector.JointLocalAngularVelocities[joint.index];

            angVelAcc = w + math.mul(q, angVelAcc);
            joint = GetParent(joint);
        }

        return angVelAcc;
    }
}
}