using System.Collections.Generic;
using System.IO;
using MotionMatching;
using Unity.Mathematics;
using UnityEngine;

namespace MotionField
{
/// <summary>
/// Everything a motion field needs: which animations form the database, and the hyperparameters
/// used to train and run the value function over it.
///
/// This implements <see cref="IPoseSetSource"/> so it can produce its own .mmskeleton/.mmpose
/// database without being a <see cref="MotionMatchingData"/>. It deliberately carries none of the
/// trajectory/pose feature machinery -- a motion field searches on full-body pose and velocity, so
/// the feature set a Motion Matching query needs has no meaning here.
/// </summary>
[CreateAssetMenu(fileName = "MotionFieldConfig", menuName = "MotionField/MotionFieldConfig")]
public class MotionFieldConfig : ScriptableObject, IPoseSetSource
{
    [Header("Animation Database")]
    [Tooltip("Clips forming the motion field. The first one supplies the skeleton.")]
    public List<AnnotatedAnimationClip> animationClips = new();

    [Tooltip("Local axis of the hips pointing forward. Orients the simulation bone, which is what " +
             "makes the character's facing direction well defined.")]
    public float3 hipsForwardLocalVector = new(0, 0, 1);

    [Tooltip("Foot speed below which a toe counts as planted.")]
    public float contactVelocityThreshold = 0.15f;

    [Tooltip("Maps animation channel names onto Mecanim bones. Pose extraction needs the toes.")]
    public List<MotionMatchingData.JointToMecanim> animationChannelToMecanim = new();

    [Header("Python Runtime")]
    [Tooltip("Full path to the CPython shared library, e.g. .../python313.dll. Leave empty to use " +
             "the PYTHONNET_PYDLL environment variable.")]
    public string pythonDllPath = "";

    [Tooltip("Virtual environment whose site-packages holds numpy, scipy and torch.")]
    public string pythonVenvPath = "";

    [Header("Motion Field")]
    [Tooltip("Neighbours considered per step. Also the number of candidate actions, since each " +
             "action emphasises one neighbour.")]
    [Min(2)] public int kNeighbors = 15;

    [Tooltip("Per-step pull back toward the nearest database state. Stops the field drifting into " +
             "regions it has no data for; too high and it just replays the database.")]
    [Range(0f, 1f)] public float tugRatio = 0.1f;

    [Tooltip("Weight on the joint-position half of the similarity metric.")]
    public float posWeight = 0.2f;

    [Tooltip("Weight on the joint-velocity half of the similarity metric.")]
    public float velWeight = 0.9f;

    [Header("Training")]
    [Tooltip("Goal headings the value function is fitted over, spanning a full turn.")]
    [Min(3)] public int thetaCount = 17;

    [Tooltip("Maximum Bellman iterations. Training stops early once the residual settles.")]
    [Min(1)] public int epochs = 300;

    [Tooltip("Discount factor. 0.99 gives roughly a 100-frame planning horizon.")]
    [Range(0.5f, 0.9999f)] public float gamma = 0.99f;

    public enum ComputeDevice { Auto, Cuda, Cpu }

    [Tooltip("Auto prefers CUDA when torch reports it available.")]
    public ComputeDevice device = ComputeDevice.Auto;

    [Tooltip("Queries per batched k-NN chunk. Bounds VRAM; lower it if training runs out of memory.")]
    [Min(1)] public int knnChunk = 64;

    [Tooltip("Database states integrated per precompute chunk. Bounds host RAM.")]
    [Min(1)] public int stateChunk = 512;

    private PoseSet _poseSet;

    // --- IPoseSetSource ---------------------------------------------------------------------

    public List<AnnotatedAnimationClip> AnimationClips => animationClips;
    public float3 HipsForwardLocalVector => hipsForwardLocalVector;
    public float ContactVelocityThreshold => contactVelocityThreshold;

    /// <summary>No trajectory features, so every pose is usable.</summary>
    public int MaximumFramesPrediction => 0;

    public bool TryGetMecanimBone(string jointName, out HumanBodyBones bone)
    {
        for (int i = 0; i < animationChannelToMecanim.Count; i++)
        {
            if (animationChannelToMecanim[i].name != jointName) continue;
            bone = animationChannelToMecanim[i].mecanimBone;
            return true;
        }

        bone = HumanBodyBones.LastBone;
        return false;
    }

    /// <summary>
    /// Where the database and the trained value function live. A folder of its own rather than
    /// MMDatabases, which the Motion Matching inspector owns and regenerates.
    /// </summary>
    public string GetAssetPath()
    {
        string path = Path.Combine(Application.streamingAssetsPath, "MotionFields", name);
#if UNITY_EDITOR
        if (!Directory.Exists(path))
        {
            Directory.CreateDirectory(path);
        }
#endif
        return path;
    }

    // --- Motion field artefacts -------------------------------------------------------------

    /// <summary>The trained value function, shipped with the player.</summary>
    public string GetValueFunctionPath() => Path.Combine(GetAssetPath(), name + ".mffield.npz");

    /// <summary>
    /// Rebuild cache for the precomputed transitions. Tens of megabytes and useless at runtime, so
    /// it goes in Library/ where Unity never imports it and no build picks it up.
    /// </summary>
    public string GetTablesCachePath() =>
        Path.GetFullPath(Path.Combine(Application.dataPath, "..", "Library", "MotionField",
            name + ".mftables.npz"));

    public string DeviceName => device switch
    {
        ComputeDevice.Cuda => "cuda",
        ComputeDevice.Cpu => "cpu",
        _ => "auto"
    };

    // --- Pose database ----------------------------------------------------------------------

    public PoseSet GetOrImportPoseSet()
    {
        if (_poseSet != null) return _poseSet;

        PoseSerializer serializer = new PoseSerializer();
        if (serializer.Deserialize(GetAssetPath(), name, this, out PoseSet poseSet))
        {
            _poseSet = poseSet;
            return _poseSet;
        }

        Debug.LogWarning($"[MotionField] No serialized pose set for '{name}'. Extracting at runtime. " +
                         "Press Generate Pose Database on the config to avoid this.");
        ImportPoseSet();
        return _poseSet;
    }

    /// <summary>Extract the pose database from the animation clips, in memory.</summary>
    public void ImportPoseSet()
    {
        Debug.Assert(animationClips.Count > 0, $"[MotionField] '{name}' has no animation clips.");

        // Bakes the Mecanim bone type onto each skeleton joint. Pose extraction locates the toes
        // through it, so it has to run before Extract.
        foreach (var clip in animationClips)
        {
            clip.UpdateMecanimInformation(this);
        }

        _poseSet = new PoseSet(this);
        _poseSet.SetSkeletonFromBvh(animationClips[0].Skeleton);

        for (int i = 0; i < animationClips.Count; i++)
        {
            if (!PoseExtractor.Extract(animationClips[i], _poseSet, this))
            {
                Debug.LogWarning($"[MotionField] Failed to extract poses from clip {i} of '{name}'.");
            }
        }

        _poseSet.ConvertTagsToNativeArrays();
    }

    /// <summary>Drop the cached pose set so the next access re-reads it from disk.</summary>
    public void InvalidatePoseSet()
    {
        _poseSet = null;
    }
}
}
