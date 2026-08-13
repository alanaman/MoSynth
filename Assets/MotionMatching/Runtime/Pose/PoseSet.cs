using System;
using System.Collections.Generic;
using AnimationTools;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{
/// <summary>
/// Stores the full pose representation of all poses for Motion Matching.
/// Poses are stored as flat <see cref="PoseBuffer"/> frames in a single
/// <see cref="PoseSequence"/> over a <see cref="AnimationTools.SkeletonAsset"/> whose bone
/// index 0 is the SimulationBone; <see cref="GetPose"/> converts back to
/// <see cref="PoseVector"/> through a <see cref="PoseVectorAdapter"/>.
/// </summary>
public class PoseSet
{
    // Public ---
    public float FrameTime { get; private set; }
    public int NumberPoses => _poseCount;
    public int NumberClips => _clips.Count;
    public int NumberTags => _tags.Count;

    /// <summary>The pose skeleton: SimulationBone at index 0, bone ids = index + 1.</summary>
    public SkeletonAsset SkeletonAsset => _skeletonAsset;

    /// <summary>Layout of the buffers returned by <see cref="GetPoseBuffer"/>.</summary>
    public PoseLayout PoseLayout => _adapter?.Layout;

    /// <summary>Reads the foot-contact Bool channels of a <see cref="GetPoseBuffer"/> frame.</summary>
    public BoolHandle LeftFootContactHandle => _leftFootContactHandle;
    public BoolHandle RightFootContactHandle => _rightFootContactHandle;

    public readonly int MaximumFramesPrediction; // Number of prediction frames of the longest trajectory feature

    // Private ---
    private readonly List<AnimationClip> _clips;
    private readonly List<AnimationTag> _tags;
    private readonly Dictionary<string, int> _tagNameToIndex;

    private SkeletonAsset _skeletonAsset;
    private PoseVectorAdapter _adapter;
    // Capacity view over Domain-backed storage; only the first _poseCount frames hold poses.
    private PoseSequence _poseStorage;
    private int _poseCount;
    private BoolHandle _leftFootContactHandle;
    private BoolHandle _rightFootContactHandle;

    public PoseSet(IPoseSetSource source) : this(source.MaximumFramesPrediction)
    {
    }

    public PoseSet(int maximumFramesPrediction)
    {
        _clips = new List<AnimationClip>();
        _tags = new List<AnimationTag>();
        _tagNameToIndex = new Dictionary<string, int>();
        FrameTime = -1.0f;
        MaximumFramesPrediction = maximumFramesPrediction;
    }

    /// <summary>
    /// Set skeleton from BVH. Adds simulation bone as root joint
    /// </summary>
    public void SetSkeletonFromBvh(SkeletonAsset bvhSkeleton)
    {
        var bones = new List<Bone>(bvhSkeleton.BoneCount + 1)
        {
            new()
            {
                id = 1,
                name = "SimulationBone",
                parentIndex = -1,
                restLocalPosition = float3.zero,
                restLocalRotation = quaternion.identity,
                humanBone = HumanBodyBones.LastBone
            }
        };
        // Add bones, shifting indices by 1 so SimulationBone is 0; the BVH root hangs off it.
        for (var i = 0; i < bvhSkeleton.BoneCount; ++i)
        {
            var bone = bvhSkeleton.GetBone(i);
            bones.Add(new Bone
            {
                id = i + 2,
                name = bone.name,
                parentIndex = i == 0 ? 0 : bone.parentIndex + 1,
                restLocalPosition = bone.restLocalPosition,
                restLocalRotation = quaternion.identity,
                humanBone = bone.humanBone
            });
        }

        SetSkeleton(CreateRuntimeSkeletonAsset(bones));
    }

    /// <summary>
    /// Adopts an already-complete skeleton (SimulationBone expected at index 0) and rebuilds
    /// the adapter/layout over it. Pose storage starts empty: every real caller sets the
    /// skeleton exactly once, before adding any pose.
    /// </summary>
    public void SetSkeleton(SkeletonAsset skeletonAsset)
    {
        Debug.Assert(_poseCount == 0, "Setting the skeleton discards previously stored poses");
        _skeletonAsset = skeletonAsset;
        RebuildAdapterAndChannels();
    }

    /// <summary>
    /// Creates an in-memory <see cref="AnimationTools.SkeletonAsset"/> for runtime use.
    /// HideAndDontSave + Domain-backed pose storage share one lifetime model: both live until
    /// domain unload. <see cref="Dispose"/> deliberately leaves the asset alone because pose
    /// sets are shared (see MotionMatchingData.GetOrImportPoseSet).
    /// </summary>
    public static SkeletonAsset CreateRuntimeSkeletonAsset(List<Bone> bones, string name = "PoseSetSkeleton")
    {
        var asset = ScriptableObject.CreateInstance<SkeletonAsset>();
        asset.name = name;
        asset.hideFlags = HideFlags.HideAndDontSave;
        asset.SetBones(bones, bones.Count + 1);
        return asset;
    }

    private void RebuildAdapterAndChannels()
    {
        var boneCount = _skeletonAsset.BoneCount;
        var mmToAsset = new int[boneCount];
        var assetToMm = new int[boneCount];
        for (var i = 0; i < boneCount; i++)
        {
            mmToAsset[i] = i;
            assetToMm[i] = i;
        }
        var map = new SkeletonMap { MmToAsset = mmToAsset, AssetToMm = assetToMm, IsIdentity = true };

        var leftContactBone = PickContactBoneIndex(HumanBodyBones.LeftToes, HumanBodyBones.LeftFoot, 0);
        var rightContactBone = PickContactBoneIndex(HumanBodyBones.RightToes, HumanBodyBones.RightFoot,
            _skeletonAsset.BoneCount - 1);
        // The layout rejects duplicate (bone, type, usage) triples, so a skeleton missing one
        // side must still end up with two distinct host bones.
        if (rightContactBone == leftContactBone)
        {
            rightContactBone = leftContactBone == 0 ? _skeletonAsset.BoneCount - 1 : 0;
        }

        var contactChannels = new List<ChannelDescriptor>
        {
            new() { boneId = _skeletonAsset.GetBone(leftContactBone).id, type = ChannelType.Bool, usage = ChannelUsage.Contact },
            new() { boneId = _skeletonAsset.GetBone(rightContactBone).id, type = ChannelType.Bool, usage = ChannelUsage.Contact }
        };

        _adapter = new PoseVectorAdapter(_skeletonAsset, map, contactChannels);
        _leftFootContactHandle = _adapter.Layout.BindBool(
            BoneRef(leftContactBone), ChannelUsage.Contact);
        _rightFootContactHandle = _adapter.Layout.BindBool(
            BoneRef(rightContactBone), ChannelUsage.Contact);

        _poseStorage = null;
        _poseCount = 0;
    }

    private BoneReference BoneRef(int boneIndex)
    {
        var bone = _skeletonAsset.GetBone(boneIndex);
        return new BoneReference(bone.id, bone.name);
    }

    private int PickContactBoneIndex(HumanBodyBones toes, HumanBodyBones foot, int fallbackIndex)
    {
        if (_skeletonAsset.TryFindByHumanBone(toes, out var index)) return index;
        if (_skeletonAsset.TryFindByHumanBone(foot, out index)) return index;
        return fallbackIndex;
    }

    /// <summary>
    /// Add the animation clip to the current pose set
    /// Returns true if the clip was added, false if the skeleton is not compatible and the clip was not added
    /// </summary>
    public bool AddClip(PoseVector[] poses, float frameTime, List<AnnotatedAnimationClip.Tag> tags)
    {
        // Check if the skeleton and frameTime are compatible
        Debug.Assert(_skeletonAsset != null, "Skeleton should be set first. Use SetSkeleton(...)");
        if (FrameTime == -1.0f) FrameTime = frameTime;
        Debug.Assert(math.abs(FrameTime - frameTime) < 0.001f, "Frame time should be the same for all clips");

        // Add poses
        int start = _poseCount;
        int nPoses = poses.Length;

        _clips.Add(new AnimationClip(start, start + nPoses, frameTime));
        EnsureCapacity(_poseCount + nPoses);
        for (var i = 0; i < nPoses; i++)
        {
            WritePose(_poseCount + i, poses[i]);
        }
        _poseCount += nPoses;

        foreach (var tag in tags)
        {
            AddTag(_clips.Count - 1, tag);
        }

        return true;
    }

    public void AddClip(PoseVector pose)
    {
        EnsureCapacity(_poseCount + 1);
        WritePose(_poseCount, pose);
        _poseCount++;
    }

    public void SetPoseCapacity(uint numPoses)
    {
        EnsureCapacity((int)numPoses);
    }

    public void SetClipCapacity(uint count)
    {
        _clips.Capacity = (int)count;
    }

    /// <summary>
    /// Grows pose storage to hold at least <paramref name="poseCapacity"/> frames. Storage is
    /// Domain-backed (never disposed, freed on domain unload); an outgrown buffer is simply
    /// abandoned to the domain after its contents are copied over.
    /// </summary>
    private void EnsureCapacity(int poseCapacity)
    {
        Debug.Assert(_adapter != null, "Skeleton should be set first. Use SetSkeleton(...)");
        if (poseCapacity <= 0) return;
        if (_poseStorage != null && poseCapacity <= _poseStorage.FrameCount) return;

        var newCapacity = math.max(poseCapacity, _poseStorage == null ? 64 : _poseStorage.FrameCount * 2);
        var floatCount = _adapter.Layout.FloatCount;
        var newData = new NativeArray<float>(newCapacity * floatCount, Allocator.Domain, NativeArrayOptions.ClearMemory);
        if (_poseStorage != null)
        {
            NativeArray<float>.Copy(_poseStorage.Data, newData, _poseCount * floatCount);
        }

        _poseStorage = new PoseSequence(_adapter.Layout, newData);
    }

    private void WritePose(int poseIndex, in PoseVector pose)
    {
        var frame = _poseStorage.GetFrame(poseIndex);
        _adapter.Write(pose, frame);
        frame.SetBool(_leftFootContactHandle, pose.leftFootContact);
        frame.SetBool(_rightFootContactHandle, pose.rightFootContact);
    }

    /// <summary>
    /// Add a tag to the current pose set
    /// The corresponding animation clip should be added before using AddTag(...)
    /// </summary>
    private void AddTag(int animationClip, AnnotatedAnimationClip.Tag dataTag)
    {
        // Tag Index
        if (!_tagNameToIndex.TryGetValue(dataTag.name, out int tagIndex))
        {
            tagIndex = _tags.Count;
            _tagNameToIndex[dataTag.name] = tagIndex;
            _tags.Add(new AnimationTag(dataTag.name));
        }

        // Write tag ranges
        AnimationTag animationTag = _tags[tagIndex];
        int frameOffset = _clips[animationClip].Start;
        for (int i = 0; i < dataTag.start.Length; ++i)
        {
            animationTag.AddRange(dataTag.start[i] + frameOffset, dataTag.end[i] + frameOffset);
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
        EnsureCapacity(_poseCount + poses.Length);
        for (var i = 0; i < poses.Length; i++)
        {
            WritePose(_poseCount + i, poses[i]);
        }
        _poseCount += poses.Length;
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
        Debug.Assert(poseIndex >= 0 && poseIndex < _poseCount, "Pose index out of range");
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
        var frame = GetPoseBuffer(poseIndex);
        pose = new PoseVector(_skeletonAsset.BoneCount);
        _adapter.Read(frame, ref pose);
        pose.leftFootContact = frame.GetBool(_leftFootContactHandle);
        pose.rightFootContact = frame.GetBool(_rightFootContactHandle);
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
    /// View over one stored pose — never Dispose it; it aliases this set's storage. Contacts
    /// are readable through <see cref="LeftFootContactHandle"/>/<see cref="RightFootContactHandle"/>.
    /// </summary>
    public PoseBuffer GetPoseBuffer(int poseIndex)
    {
        Debug.Assert(poseIndex >= 0 && poseIndex < _poseCount, "Pose index out of range");
        return _poseStorage.GetFrame(poseIndex);
    }

    /// <summary>
    /// Returns the position of each joint in world space after applying FK using the pose.
    /// worldJoints has size SkeletonAsset.BoneCount
    /// </summary>
    public NativeArray<float3> GetWorldPositions(PoseVector pose, quaternion inverseRotAnimationSpace,
        float3 posAnimationSpace, quaternion rotWorld, float3 posWorld)
    {

        // animation space to local space
        float3 localSpacePos = math.mul(inverseRotAnimationSpace, pose.jointLocalPositions[0] - posAnimationSpace);
        quaternion localSpaceRot = math.mul(inverseRotAnimationSpace, pose.jointLocalRotations[0]);
        // local space to world space
        float3 simulationBonePos = math.mul(rotWorld, localSpacePos) + posWorld;
        quaternion simulationBoneRot = math.mul(rotWorld, localSpaceRot);

        var simulationBoneTransform = Matrix4x4.TRS(simulationBonePos, simulationBoneRot, Vector3.one);
        return GetWorldPositions(pose, simulationBoneTransform);
    }

    public NativeArray<float3> GetWorldPositions(PoseVector pose, Matrix4x4 simulationBoneTransform)
    {
        var boneCount = _skeletonAsset.BoneCount;
        var skeletonData = _skeletonAsset.GetSkeletonData();
        Span<Matrix4x4> localToWorldRes = stackalloc Matrix4x4[boneCount];
        localToWorldRes[0] = simulationBoneTransform;
        for (int i = 1; i < boneCount; i++)
        {
            localToWorldRes[i] = Matrix4x4.identity;
        }

        for (int i = 1; i < boneCount; i++)
        {
            Matrix4x4 current = Matrix4x4.TRS(pose.jointLocalPositions[i],
                pose.jointLocalRotations[i], Vector3.one);
            localToWorldRes[i] = localToWorldRes[skeletonData.ParentIndices[i]] * current;
        }

        var worldJoints = new NativeArray<float3>(boneCount, Allocator.Temp);
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
        // Pose storage and the mirror SkeletonAsset are Domain-lifetime by design: pose sets
        // are shared (MotionMatchingData caches one), so Dispose only releases the tags.
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
            _startRanges = new NativeArray<int>(_startRangesList.ToArray(), Allocator.Domain);
            _endRanges = new NativeArray<int>(_endRangesList.ToArray(), Allocator.Domain);

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
