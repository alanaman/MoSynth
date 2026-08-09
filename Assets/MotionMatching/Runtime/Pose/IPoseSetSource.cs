using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{
/// <summary>
/// Everything the pose-database pipeline needs from the asset it is extracting.
///
/// <see cref="PoseSet"/>, <see cref="PoseExtractor"/> and <see cref="PoseSerializer"/> used to take
/// a <see cref="MotionMatchingData"/> directly, which meant anything wanting a pose database also
/// had to be a full Motion Matching asset -- trajectory features, pose features, T-Pose, mecanim
/// map and all. Only a handful of those members actually reach the pose path, and this interface is
/// exactly that handful, so a consumer such as MotionField's config can supply a database without
/// dragging in the feature-set machinery it never uses.
///
/// Note this lives in the MotionMatching runtime assembly rather than beside its other
/// implementor: Assembly-CSharp can reference MotionMatching, but not the other way round.
/// </summary>
public interface IPoseSetSource
{
    /// <summary>Asset name. Doubles as the database folder and file base name.</summary>
    string name { get; }

    /// <summary>Clips to extract, in order. The first one supplies the skeleton.</summary>
    List<AnnotatedAnimationClip> AnimationClips { get; }

    /// <summary>Local axis of the hips pointing forward, used to orient the simulation bone.</summary>
    float3 HipsForwardLocalVector { get; }

    /// <summary>Foot speed below which a toe counts as planted.</summary>
    float ContactVelocityThreshold { get; }

    /// <summary>
    /// Frames of lookahead the longest trajectory feature needs, so poses too close to the end of a
    /// clip can be excluded from prediction. Zero when the consumer has no trajectory features.
    /// </summary>
    int MaximumFramesPrediction { get; }

    /// <summary>
    /// Maps an animation channel name onto a Mecanim bone. Pose extraction needs this to locate the
    /// toes for contact detection.
    /// </summary>
    bool TryGetMecanimBone(string jointName, out HumanBodyBones bone);

    /// <summary>Directory the serialized database lives in. Created if missing (editor only).</summary>
    string GetAssetPath();
}
}
