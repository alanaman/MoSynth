using System;
using System.Collections.Generic;
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
        public Vector3 localOffset;
        public HumanBodyBones type;

        public Joint(string name, int index, int parentIndex, Vector3 localOffset)
        {
            this.name = name;
            this.index = index;
            this.parentIndex = parentIndex;
            this.localOffset = localOffset;
            type = HumanBodyBones.LastBone;
        }

        public Joint(string name, int index, int parentIndex, Vector3 localOffset, HumanBodyBones type)
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
                   localOffset == other.localOffset && type == other.type;
        }
    }
}
}