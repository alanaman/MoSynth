using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using System;
using UnityEngine.Serialization;

namespace MotionMatching
{
    using Joint = Skeleton.Joint;

    /// <summary>
    /// Stores the BVH animation data in Unity format.
    /// </summary>
    public class BvhAnimation : ScriptableObject
    {
        [SerializeField] private float frameTime;
        public float FrameTime => frameTime;

        [SerializeField] private Skeleton skeleton = new();
        public Skeleton Skeleton => skeleton;
        
        public List<EndSite> EndSites { get; private set; } = new();
        
        [SerializeField]
        private Frame[] frames;
        public Frame[] Frames => frames;
        

        public void SetFrameTime(float inFrameTime)
        {
            this.frameTime = inFrameTime;
        }

        public void InitFrames(int numberFrames)
        {
            frames = new Frame[numberFrames];
        }

        public void AddFrame(int index, Frame frame)
        {
            Frames[index] = frame;
        }

        public void AddJoint(Joint joint)
        {
            Skeleton.AddJoint(joint);
        }

        internal void AddEndSite(EndSite endSite)
        {
            EndSites.Add(endSite);
        }

        public void UpdateMecanimInformation(MotionMatchingData motionMatchingData)
        {
            for (int i = 0; i < Skeleton.Joints.Count; i++)
            {
                Joint joint = Skeleton.Joints[i];
                if (motionMatchingData.GetMecanimBone(joint.Name, out HumanBodyBones bone))
                {
                    joint.Type = bone;
                    Skeleton.Joints[i] = joint;
                }
            }
        }

        /// <summary>
        /// Apply forward kinematics to obtain the quaternion rotating from the local
        /// coordinate system of the joint to the world coordinate system.
        /// </summary>
        public quaternion GetWorldRotation(Joint joint, int frameIndex)
        {
            Frame frame = Frames[frameIndex];
            quaternion worldRot = quaternion.identity;

            while (joint.Index != 0) // while not root
            {
                worldRot = frame.localRotations[joint.Index] * worldRot;
                joint = Skeleton.GetParent(joint);
            }
            worldRot = frame.localRotations[0] * worldRot; // root

            return worldRot;
        }

        public struct EndSite
        {
            public int ParentIndex;
            public Vector3 Offset;

            public EndSite(int parentIndex, Vector3 offset)
            {
                ParentIndex = parentIndex;
                Offset = offset;
            }
        }

        [Serializable]
        public struct Frame
        {
            [FormerlySerializedAs("RootMotion")] public Vector3 rootMotion;
            [FormerlySerializedAs("LocalRotations")] public Quaternion[] localRotations;

            public Frame(Vector3 rootMotion, Quaternion[] localRotations)
            {
                this.rootMotion = rootMotion;
                this.localRotations = localRotations;
            }
        }
    }
}