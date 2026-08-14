using System;
using System.Collections.Generic;
using AnimationTools;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{
/// <summary>
/// Read-only pass-through stage that estimates foot-style contact from a remapped
/// <see cref="PoseBuffer"/> and draws it as a gizmo-free debug marker. Exists to prove the
/// AnimationTools pose-remap path end to end; it never mutates the pose.
/// </summary>
[Serializable]
public class ContactVisualizerStage : MoSynthStage
{
    [SerializeField] private SkeletonAsset skeleton;
    [BoneFrom(nameof(skeleton))] [SerializeField] private List<BoneReference> contactBones = new();
    [SerializeField] private float contactVelocityThreshold = 0.15f;
    [SerializeField] private float markerSize = 0.08f;

    private MotionSynthesisComponent _component;
    private SkeletonAsset _effectiveSkeleton;
    private SkeletonMap _map;
    private PoseBuffer _buffer;
    private SkeletonData _skeletonData;
    private bool _inert;

    private BoolHandle[] _contactHandles;
    private int[] _contactAssetBoneIndices;
    private int[] _contactMmJointIndices;

    public override SkeletonAsset GetSkeleton(SkeletonAsset inSkeleton) => inSkeleton;

    public override void Init(MotionSynthesisComponent motionSynthesisComponent)
    {
        _component = motionSynthesisComponent;

        var effectiveSkeleton = skeleton != null ? skeleton : motionSynthesisComponent.PoseSkeleton;
        if (effectiveSkeleton == null)
        {
            Debug.LogWarning($"[ContactVisualizerStage] \"{motionSynthesisComponent.name}\": no skeleton asset assigned (neither the stage's own nor the component's PoseSkeleton). Disabling.");
            _inert = true;
            return;
        }

        var map = effectiveSkeleton == motionSynthesisComponent.PoseSkeleton && motionSynthesisComponent.PoseSkeletonMap != null
            ? motionSynthesisComponent.PoseSkeletonMap
            : SkeletonMap.Build(motionSynthesisComponent.Skeleton, effectiveSkeleton);

        if (map == null)
        {
            Debug.LogWarning($"[ContactVisualizerStage] \"{motionSynthesisComponent.name}\": could not map the MotionMatching skeleton onto \"{effectiveSkeleton.name}\". Disabling.");
            _inert = true;
            return;
        }

        var extraChannels = new List<ChannelDescriptor>();
        foreach (var boneRef in contactBones)
        {
            if (!boneRef.IsSet) continue;
            extraChannels.Add(new ChannelDescriptor
            {
                boneId = boneRef.BoneId, type = ChannelType.Bool, space = ChannelSpace.Character,
                representation = RotationRepresentation.Quaternion, usage = ChannelUsage.Contact
            });
        }

        _effectiveSkeleton = effectiveSkeleton;
        _map = map;

        var channels = PoseLayoutBuilder.BuildFullPoseChannels(effectiveSkeleton);
        channels.AddRange(extraChannels);
        var layout = PoseLayout.Build(effectiveSkeleton, channels);
        _buffer = PoseBuffer.Allocate(layout, Allocator.Persistent);
        _skeletonData = effectiveSkeleton.GetSkeletonData();

        var handles = new List<BoolHandle>();
        var assetIndices = new List<int>();
        var mmJointIndices = new List<int>();

        foreach (var boneRef in contactBones)
        {
            if (!boneRef.IsSet) continue;

            var assetIndex = boneRef.ResolveIndex(effectiveSkeleton);
            var mmJointIndex = assetIndex >= 0 ? map.AssetToMm[assetIndex] : -1;
            if (assetIndex < 0 || mmJointIndex < 0)
            {
                Debug.LogWarning($"[ContactVisualizerStage] \"{motionSynthesisComponent.name}\": contact bone \"{boneRef.CachedName}\" has no corresponding MotionMatching joint; skipping.");
                continue;
            }

            handles.Add(layout.BindBool(boneRef, ChannelUsage.Contact));
            assetIndices.Add(assetIndex);
            mmJointIndices.Add(mmJointIndex);
        }

        _contactHandles = handles.ToArray();
        _contactAssetBoneIndices = assetIndices.ToArray();
        _contactMmJointIndices = mmJointIndices.ToArray();
    }

    public override bool Apply(PoseBuffer pose, float deltaTime)
    {
        if (_inert) return true;

        var srcPositions = pose.Positions;
        var srcRotations = pose.Rotations;
        var srcVelocities = pose.Velocities;
        var srcAngularVelocities = pose.AngularVelocities;
        var dstPositions = _buffer.Positions;
        var dstRotations = _buffer.Rotations;
        var dstVelocities = _buffer.Velocities;
        var dstAngularVelocities = _buffer.AngularVelocities;
        var boneCount = _effectiveSkeleton.BoneCount;
        for (var i = 0; i < boneCount; i++)
        {
            var mmIndex = _map.AssetToMm[i];
            if (mmIndex < 0)
            {
                var bone = _effectiveSkeleton.GetBone(i);
                dstPositions[i] = bone.restLocalPosition;
                dstRotations[i] = bone.restLocalRotation;
                dstVelocities[i] = float3.zero;
                dstAngularVelocities[i] = float3.zero;
                continue;
            }
            dstPositions[i] = srcPositions[mmIndex];
            dstRotations[i] = srcRotations[mmIndex];
            dstVelocities[i] = srcVelocities[mmIndex];
            dstAngularVelocities[i] = srcAngularVelocities[mmIndex];
        }

        for (var k = 0; k < _contactHandles.Length; k++)
        {
            var velocity = PoseFK.CharacterVelocity(_buffer, _skeletonData, _contactAssetBoneIndices[k]);
            var contact = math.length(velocity) < contactVelocityThreshold;
            _buffer.SetBool(_contactHandles[k], contact);

            var worldPosition = _component.SkeletonTransforms[_contactMmJointIndices[k]].position;
            DrawContactMarker(worldPosition, contact);
        }

        return true;
    }

    private void DrawContactMarker(float3 position, bool contact)
    {
        var color = contact ? Color.green : Color.red;
        var half = markerSize * 0.5f;
        Debug.DrawLine(position - new float3(half, 0f, 0f), position + new float3(half, 0f, 0f), color, 0f);
        Debug.DrawLine(position - new float3(0f, half, 0f), position + new float3(0f, half, 0f), color, 0f);
        Debug.DrawLine(position - new float3(0f, 0f, half), position + new float3(0f, 0f, half), color, 0f);
    }

    public override void OnValidate()
    {
        contactVelocityThreshold = math.max(0f, contactVelocityThreshold);
        markerSize = math.max(0f, markerSize);
    }

    public override void OnDestroy()
    {
        if (_buffer.IsCreated) _buffer.Dispose();
    }
}
}
