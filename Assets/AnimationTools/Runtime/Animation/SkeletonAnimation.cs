using System;
using Unity.Collections;
using UnityEngine;

namespace AnimationTools
{
/// <summary>
/// A clip-backed skeletal animation: wraps a <see cref="UnityEngine.AnimationClip"/> together
/// with a reference rig, whose Transform hierarchy IS the skeleton the clip is sampled against.
/// Frames are lazily baked (see <see cref="AnimationClipBaker"/>) into the AnimationTools pose
/// format on first access to <see cref="PoseSequence"/> or <see cref="Skeleton"/>.
/// </summary>
public class SkeletonAnimation : ScriptableObject
{
    [SerializeField] private AnimationClip clip;
    [SerializeField] private GameObject rig;
    [BoneFrom(nameof(rig))] [SerializeField] private BoneTransform rootBone = new();

    public AnimationClip Clip => clip;
    public GameObject Rig => rig;
    public bool HasClip => clip != null;

    /// <summary>
    /// Resolves the skeleton's bone 0. Resolution order: the explicitly assigned
    /// <see cref="rootBone"/> — its Transform, else its cached bone name looked up under
    /// <see cref="rig"/>, so the reference survives a build that strips the rig objects; else, when
    /// <see cref="rig"/> has exactly one child, that child; else the first descendant of
    /// <see cref="rig"/> (rig root excluded) whose name ends with "Hips", found by depth-first
    /// search; else null.
    /// </summary>
    public Transform RootBone
    {
        get
        {
            if (rootBone != null && rootBone.Transform != null) return rootBone.Transform;
            if (rig == null) return null;

            if (rootBone != null && !string.IsNullOrEmpty(rootBone.BoneName))
            {
                var named = FindRecursive(rig.transform, child => child.name == rootBone.BoneName);
                if (named != null) return named;
            }

            if (rig.transform.childCount == 1) return rig.transform.GetChild(0);

            // Suffix, not equality: rigs exported with a namespace prefix name the bone "Model:Hips".
            return FindRecursive(rig.transform,
                child => child.name.EndsWith("Hips", StringComparison.OrdinalIgnoreCase));
        }
    }

    private static Transform FindRecursive(Transform transform, Func<Transform, bool> predicate)
    {
        for (var i = 0; i < transform.childCount; i++)
        {
            var child = transform.GetChild(i);
            if (predicate(child)) return child;

            var found = FindRecursive(child, predicate);
            if (found != null) return found;
        }

        return null;
    }

    public float FrameTime => clip != null ? 1f / clip.frameRate : 0f;

    // Plain arithmetic; must not materialize the sequence, since OnValidate and inspectors call
    // this every repaint.
    public int FrameCount => clip == null ? 0 : Mathf.Max(1, Mathf.RoundToInt(clip.length * clip.frameRate) + 1);

    [NonSerialized] private Skeleton _skeleton;

    public Skeleton Skeleton
    {
        get
        {
            if (_skeleton != null) return _skeleton;

            var resolvedRootBone = RootBone;
            if (resolvedRootBone == null) return null;

            var skeletonRoot = new SkeletonRoot();
            skeletonRoot.SetRoot(resolvedRootBone);
            _skeleton = skeletonRoot.BuildSkeleton(name);
            return _skeleton;
        }
    }

    [NonSerialized] private PoseSequence _poseSequence;

    public PoseSequence PoseSequence
    {
        get
        {
            if (clip == null) return null;

            var skeleton = Skeleton;
            if (skeleton == null) return null;

            if (_poseSequence != null) return _poseSequence;

            var layout = PoseLayout.CreateFullPose(skeleton, false, false);
            var baked = AnimationClipBaker.Bake(clip, rig, RootBone, skeleton, FrameCount, FrameTime);
            Debug.Assert(baked.Length == FrameCount * layout.FloatCount,
                $"SkeletonAnimation \"{name}\": baked.Length ({baked.Length}) does not match frameCount * layout.FloatCount ({FrameCount * layout.FloatCount}).");

            var nativeFrameData = new NativeArray<float>(baked, Allocator.Domain);

            _poseSequence = new PoseSequence(layout, nativeFrameData);
            return _poseSequence;
        }
    }

    public PoseBuffer GetFrame(int frameIndex) => PoseSequence.GetFrame(frameIndex);

    /// <summary>Editor-helper setup for creation menus and tests: assigns the source fields
    /// directly and invalidates any cached bake.</summary>
    public void SetSource(AnimationClip inClip, GameObject inRig, Transform inRootBone)
    {
        clip = inClip;
        rig = inRig;
        rootBone = new BoneTransform(inRootBone);
        ClearRuntimeCaches();
    }

    protected void ClearRuntimeCaches()
    {
        _skeleton = null;
        _poseSequence = null;
    }

    protected virtual void OnValidate() => ClearRuntimeCaches();
}
}