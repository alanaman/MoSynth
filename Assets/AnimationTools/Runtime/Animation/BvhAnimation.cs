using System;
using System.Collections.Generic;
using Unity.Collections;
using UnityEngine;

namespace AnimationTools
{
/// <summary>
/// An imported BVH animation, stored as per-frame float data in the AnimationTools pose format.
/// </summary>
public class BvhAnimation : ScriptableObject
{
    [SerializeField] private float frameTime;
    [SerializeField] private SkeletonAsset skeletonAsset;
    [SerializeField] private int frameCount;

    // Per-frame layout is PoseLayout.CreateFullPose(skeletonAsset, false, false): positions then
    // rotations, bone-index aligned. positions[0] holds the root motion for the frame;
    // positions[i > 0] repeat the bone rest offsets so frames are PoseFK-ready.
    [HideInInspector] [SerializeField] private float[] frameData;

    public float FrameTime => frameTime;
    public SkeletonAsset Skeleton => skeletonAsset;

    // Plain field read; must not materialize the sequence.
    public int FrameCount => frameCount;

    [NonSerialized] private PoseSequence _poseSequence;
    [NonSerialized] private NativeArray<float> _nativeFrameData;
    [NonSerialized] private int _poseSequenceSkeletonVersion = -1;

    public PoseSequence PoseSequence
    {
        get
        {
            // Assets imported by an older importer version deserialize with these empty.
            if (skeletonAsset == null || frameData == null || frameData.Length == 0) return null;

            if (_poseSequence != null && _poseSequenceSkeletonVersion == skeletonAsset.Version) return _poseSequence;

            var layout = PoseLayout.CreateFullPose(skeletonAsset, false, false);
            Debug.Assert(frameData.Length == frameCount * layout.FloatCount,
                $"BvhAnimation \"{name}\": frameData.Length ({frameData.Length}) does not match frameCount * layout.FloatCount ({frameCount * layout.FloatCount}).");

            // Domain-allocated, never disposed: it lives for the domain's lifetime alongside
            // this cache, the same pattern as SkeletonAsset.GetSkeletonData. A skeleton Version
            // bump (UpdateMecanimInformation) only re-wraps this same native data with a fresh
            // layout — FloatCount is unchanged because the bone list shape is unchanged.
            if (!_nativeFrameData.IsCreated)
                _nativeFrameData = new NativeArray<float>(frameData, Allocator.Domain);

            _poseSequence = new PoseSequence(layout, _nativeFrameData);
            _poseSequenceSkeletonVersion = skeletonAsset.Version;
            return _poseSequence;
        }
    }

    public PoseBuffer GetFrame(int frameIndex) => PoseSequence.GetFrame(frameIndex);

    /// <summary>Importer-only setup: assigns the raw fields directly, no validation or copying.</summary>
    public void Initialize(SkeletonAsset skeleton, float inFrameTime, int inFrameCount, float[] inFrameData)
    {
        skeletonAsset = skeleton;
        frameTime = inFrameTime;
        frameCount = inFrameCount;
        frameData = inFrameData;
    }

    public void UpdateMecanimInformation(IPoseSetSource source)
    {
        if (skeletonAsset == null) return;

        var changed = false;
        var newBones = new List<Bone>(skeletonAsset.BoneCount);
        for (var i = 0; i < skeletonAsset.BoneCount; i++)
        {
            var bone = skeletonAsset.GetBone(i);
            if (source.TryGetMecanimBone(bone.name, out var humanBone) && bone.humanBone != humanBone)
            {
                bone.humanBone = humanBone;
                changed = true;
            }

            newBones.Add(bone);
        }

        // Skipping a no-op SetBones avoids invalidating cached SkeletonData/layouts.
        if (changed) skeletonAsset.SetBones(newBones, skeletonAsset.NextBoneId);
    }
}
}