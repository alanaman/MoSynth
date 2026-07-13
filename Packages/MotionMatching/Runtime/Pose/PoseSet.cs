using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{
using Joint = Skeleton.Joint;

/// <summary>
/// Stores the full pose representation of all poses for Motion Matching
/// </summary>
public class PoseSet
{
    // Public ---
    public Skeleton Skeleton { get; private set; }
    public float FrameTime { get; private set; }
    public int NumberPoses => _poses.Count;
    public int NumberClips => _clips.Count;
    public int NumberTags => _tags.Count;

    public readonly int MaximumFramesPrediction; // Number of prediction frames of the longest trajectory feature

    // Private ---
    private readonly List<PoseVector> _poses;
    private readonly List<AnimationClip> _clips;
    private readonly List<AnimationTag> _tags;
    private readonly Dictionary<string, int> _tagNameToIndex;

    public PoseSet(MotionMatchingData mmData)
    {
        _poses = new List<PoseVector>();
        _clips = new List<AnimationClip>();
        _tags = new List<AnimationTag>();
        _tagNameToIndex = new Dictionary<string, int>();
        FrameTime = -1.0f;
        MaximumFramesPrediction = 0;
        foreach (var t in mmData.TrajectoryFeatures)
        {
            if (t.FramesPrediction[^1] > MaximumFramesPrediction)
            {
                MaximumFramesPrediction = t.FramesPrediction[^1];
            }
        }
    }

    /// <summary>
    /// Set skeleton from BVH. Adds simulation bone as root joint
    /// </summary>
    public void SetSkeletonFromBvh(Skeleton skeleton)
    {
        Skeleton = new Skeleton();
        // Add Simulation Bone
        Joint sb = new Joint("SimulationBone", 0, -1, Vector3.zero);
        Skeleton.AddJoint(sb);
        // Add Joints (adjusting indices, now SimulationBone is 0 and all indices are shifted by 1)
        for (int i = 0; i < skeleton.Joints.Count; ++i)
        {
            Joint j = skeleton.Joints[i];
            j.index = j.index + 1;
            if (i == 0) // Root
            {
                j.parentIndex = 0;
            }
            else // Other Joints
            {
                j.parentIndex = j.parentIndex + 1;
            }

            Skeleton.Joints.Add(j);
        }
    }

    /// <summary>
    /// Set skeleton from File. The skeleton is not modified
    /// </summary>
    public void SetSkeletonFromFile(Skeleton skeleton)
    {
        Skeleton = skeleton;
    }

    /// <summary>
    /// Add the animation clip to the current pose set
    /// Returns true if the clip was added, false if the skeleton is not compatible and the clip was not added
    /// </summary>
    public bool AddClip(PoseVector[] poses, float frameTime, List<AnimationData.Tag> tags)
    {
        // Check if the skeleton and frameTime are compatible
        Debug.Assert(Skeleton != null, "Skeleton should be set first. Use SetSkeleton(...)");
        if (FrameTime == -1.0f) FrameTime = frameTime;
        Debug.Assert(math.abs(FrameTime - frameTime) < 0.001f, "Frame time should be the same for all clips");

        // Add poses
        int start = _poses.Count;
        int nPoses = poses.Length;

        _clips.Add(new AnimationClip(start, start + nPoses, frameTime));
        _poses.AddRange(poses);
        foreach (var tag in tags)
        {
            AddTag(_clips.Count - 1, tag);
        }

        return true;
    }

    public void AddClip(PoseVector pose)
    {
        _poses.Add(pose);
    }

    public void SetPoseCapacity(uint numPoses)
    {
        _poses.Capacity = (int)numPoses;
    }

    public void SetClipCapacity(uint count)
    {
        _clips.Capacity = (int)count;
    }

    /// <summary>
    /// Add a tag to the current pose set
    /// The corresponding animation clip should be added before using AddTag(...)
    /// </summary>
    private void AddTag(int animationClip, AnimationData.Tag dataTag)
    {
        // Tag Index
        if (!_tagNameToIndex.TryGetValue(dataTag.Name, out int tagIndex))
        {
            tagIndex = _tags.Count;
            _tagNameToIndex[dataTag.Name] = tagIndex;
            _tags.Add(new AnimationTag(dataTag.Name));
        }

        // Write tag ranges
        AnimationTag animationTag = _tags[tagIndex];
        int frameOffset = _clips[animationClip].Start;
        for (int i = 0; i < dataTag.Start.Length; ++i)
        {
            animationTag.AddRange(dataTag.Start[i] + frameOffset, dataTag.End[i] + frameOffset);
        }
    }

    /// <summary>
    /// Add a tag to the current pose set
    /// Used when deserializing from binary format
    /// </summary>
    public void AddTagDeserialized(string name, List<int> startRangesList, List<int> endRangesList)
    {
        _tagNameToIndex[name] = _tags.Count;
        _tags.Add(new AnimationTag(name, startRangesList, endRangesList));
    }

    /// <summary>
    /// Converts all tags-related data stored in C# data structures to NativeArrays
    /// Use this function after adding all tags with AddTag(...)
    /// </summary>
    public void ConvertTagsToNativeArrays()
    {
        foreach (AnimationTag tag in _tags)
        {
            tag.ConvertToNativeArray();
        }
    }

    /// <summary>
    /// Add the set of poses to the current pose set
    /// Used when deserializing from binary format
    /// </summary>
    public void AddClipDeserialized(PoseVector[] poses)
    {
        _poses.AddRange(poses);
    }

    /// <summary>
    /// Add the animation clip to the current clips
    /// Used when deserializing from binary format
    /// </summary>
    public void AddAnimationClipDeserialized(AnimationClip clip)
    {
        Debug.Assert(math.abs(FrameTime + 1.0f) < 0.001f || math.abs(clip.FrameTime - FrameTime) < 0.001f,
            "Mixed frame rates");
        FrameTime = clip.FrameTime;
        _clips.Add(clip);
    }

    public bool IsPoseValidForPrediction(int poseIndex)
    {
        Debug.Assert(poseIndex >= 0 && poseIndex < _poses.Count, "Pose index out of range");
        // Check the validity of the pose
        bool isPredictionSafe = true;
        for (int i = 0; i < _clips.Count && isPredictionSafe; ++i)
        {
            AnimationClip clip = _clips[i];
            if (poseIndex >= clip.Start && poseIndex < clip.End)
            {
                if (poseIndex >= clip.End - MaximumFramesPrediction) isPredictionSafe = false;
            }
        }

        return isPredictionSafe;
    }

    /// <summary>
    /// Returns the pose at the given index.
    /// Return true if the pose can be used for prediction
    /// </summary>
    public bool GetPose(int poseIndex, out PoseVector pose)
    {
        bool isPredictionSafe = IsPoseValidForPrediction(poseIndex);
        pose = _poses[poseIndex];
        return isPredictionSafe;
    }

    /// <summary>
    /// Returns the pose at the given index.
    /// Return true if the pose can be used for prediction
    /// </summary>
    public bool GetPose(int poseIndex, out PoseVector pose, out int animationClip)
    {
        animationClip = -1;
        for (int clipIdx = 0; clipIdx < _clips.Count; ++clipIdx)
        {
            if (poseIndex >= _clips[clipIdx].Start && poseIndex < _clips[clipIdx].End)
            {
                animationClip = clipIdx;
                break;
            }
        }

        Debug.Assert(animationClip != -1, "Clip index not found");
        return GetPose(poseIndex, out pose);
    }

    /// <summary>
    /// Returns the position of each joint in world space after applying FK using the pose.
    /// worldJoints has size Skeleton.Joints.Count
    /// </summary>
    public void GetWorldPositions(PoseVector pose, NativeArray<float3> worldJoints)
    {
        Span<Matrix4x4> localToWorld = stackalloc Matrix4x4[Skeleton.Joints.Count];
        for (int i = 0; i < Skeleton.Joints.Count; i++)
        {
            localToWorld[i] = Matrix4x4.identity;
        }

        for (int i = 0; i < Skeleton.Joints.Count; i++)
        {
            Joint joint = Skeleton.Joints[i];
            Matrix4x4 current = Matrix4x4.TRS(pose.JointLocalPositions[joint.index],
                pose.JointLocalRotations[joint.index], Vector3.one);
            localToWorld[joint.index] = localToWorld[joint.parentIndex] * current;
        }

        for (int i = 0; i < worldJoints.Length; i++)
        {
            worldJoints[i] = localToWorld[i].MultiplyPoint3x4(Vector3.zero);
        }
    }

    /// <summary>
    /// Returns the position of each joint in world space after applying FK using the pose.
    /// worldJoints has size Skeleton.Joints.Count
    /// </summary>
    public NativeArray<float3> GetWorldPositions(PoseVector pose, quaternion inverseRotAnimationSpace,
        float3 posAnimationSpace, quaternion rotWorld, float3 posWorld)
    {

        // animation space to local space
        float3 localSpacePos = math.mul(inverseRotAnimationSpace, pose.JointLocalPositions[0] - posAnimationSpace);
        quaternion localSpaceRot = math.mul(inverseRotAnimationSpace, pose.JointLocalRotations[0]);
        // local space to world space
        float3 simulationBonePos = math.mul(rotWorld, localSpacePos) + posWorld;
        quaternion simulationBoneRot = math.mul(rotWorld, localSpaceRot);

        var simulationBoneTransform = Matrix4x4.TRS(simulationBonePos, simulationBoneRot, Vector3.one);
        return GetWorldPositions(pose, simulationBoneTransform);
    }

    public NativeArray<float3> GetWorldPositions(PoseVector pose, Matrix4x4 simulationBoneTransform)
    {
        Span<Matrix4x4> localToWorldRes = stackalloc Matrix4x4[Skeleton.Joints.Count];
        localToWorldRes[0] = simulationBoneTransform;
        for (int i = 1; i < Skeleton.Joints.Count; i++)
        {
            localToWorldRes[i] = Matrix4x4.identity;
        }

        for (int i = 1; i < Skeleton.Joints.Count; i++)
        {
            Joint joint = Skeleton.Joints[i];
            Matrix4x4 current = Matrix4x4.TRS(pose.JointLocalPositions[joint.index],
                pose.JointLocalRotations[joint.index], Vector3.one);
            localToWorldRes[joint.index] = localToWorldRes[joint.parentIndex] * current;
        }

        var worldJoints = new NativeArray<float3>(Skeleton.Joints.Count, Allocator.Temp);
        for (int i = 0; i < worldJoints.Length; i++)
        {
            worldJoints[i] = localToWorldRes[i].MultiplyPoint3x4(Vector3.zero);
        }

        return worldJoints;
    }

    /// <summary>
    /// Returns the tag at the given index
    /// </summary>
    public AnimationTag GetTag(int index)
    {
        return _tags[index];
    }

    /// <summary>
    /// Returns the tag with the given name
    /// </summary>
    public AnimationTag GetTag(string name)
    {
        return _tags[_tagNameToIndex[name]];
    }

    /// <summary>
    /// Returns the animation clip at the given index
    /// </summary>
    public AnimationClip GetAnimationClip(int clipIndex)
    {
        Debug.Assert(clipIndex >= 0 && clipIndex < _clips.Count, "Clip index out of range");
        return _clips[clipIndex];
    }

    public void Dispose()
    {
        if (_tags != null)
        {
            foreach (AnimationTag tag in _tags)
            {
                tag.Dispose();
            }
        }
    }

    public struct AnimationClip
    {
        public int Start; // Index of the first pose in the clip
        public int End; // End is exclusive
        public float FrameTime;

        public AnimationClip(int start, int end, float frameTime)
        {
            Start = start;
            End = end;
            FrameTime = frameTime;
        }
    }

    public class AnimationTag
    {
        public readonly string Name;

        private List<int> _startRangesList; // Temporal lists until they are converted to NativeArrays
        private List<int> _endRangesList;

        private NativeArray<int> _startRanges;
        private NativeArray<int> _endRanges;

        public int NumberRanges
        {
            get { return _startRanges.Length; }
        }

        public AnimationTag(string name)
        {
            Name = name;
            _startRangesList = new List<int>();
            _endRangesList = new List<int>();
        }

        public AnimationTag(string name, List<int> startRangesList, List<int> endRangesList)
        {
            Name = name;
            _startRangesList = startRangesList;
            _endRangesList = endRangesList;
        }

        public void AddRange(int start, int end)
        {
            _startRangesList.Add(start);
            _endRangesList.Add(end);
        }

        public NativeArray<int> GetStartRanges()
        {
            return _startRanges;
        }

        public NativeArray<int> GetEndRanges()
        {
            return _endRanges;
        }

        public void ConvertToNativeArray()
        {
            _startRanges = new NativeArray<int>(_startRangesList.ToArray(), Allocator.Persistent);
            _endRanges = new NativeArray<int>(_endRangesList.ToArray(), Allocator.Persistent);

            _startRangesList = null;
            _endRangesList = null;
        }

        public void GetRange(int rangeIndex, out int start, out int end)
        {
            Debug.Assert(_startRanges.IsCreated && _endRanges.IsCreated,
                "Call first ConvertToNativeArray() before operating over the tags.");
            start = _startRanges[rangeIndex];
            end = _endRanges[rangeIndex];
        }

        public void Dispose()
        {
            if (_startRanges.IsCreated) _startRanges.Dispose();
            if (_endRanges.IsCreated) _endRanges.Dispose();
        }
    }
}
}