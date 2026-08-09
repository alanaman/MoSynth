using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using MotionMatching;
using Python.Runtime;
using UnityEditor;
using UnityEngine;

namespace MotionField.Editor
{
/// <summary>
/// Inspector for <see cref="MotionFieldConfig"/>: extract the pose database, then train the value
/// function over it.
///
/// Training runs in-process through PythonNET, synchronously on the main thread. It takes roughly
/// 20 s on a GPU for a 7.8k-state database, which is short enough that a frozen editor beats the
/// alternative: numpy and scipy hold the GIL for hundreds of milliseconds at a stretch, so a
/// background worker would stall play mode anyway, cannot be aborted (Thread.Abort is gone), and
/// hard-crashes the editor if a domain reload lands while it holds the GIL.
/// </summary>
[CustomEditor(typeof(MotionFieldConfig))]
public class MotionFieldConfigEditor : UnityEditor.Editor
{
    private MotionMatchingData _importSource;
    private bool _showImport;
    private bool _showBoneWeights = true;

    // Skeleton of the generated database, cached because OnInspectorGUI repaints constantly and
    // this comes off disk. Dropped on enable and whenever the database is regenerated.
    private Skeleton _skeleton;
    private int[] _jointDepths;
    private bool _skeletonRead;

    private void OnEnable()
    {
        InvalidateSkeleton();
    }

    private void InvalidateSkeleton()
    {
        _skeleton = null;
        _jointDepths = null;
        _skeletonRead = false;
    }

    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        var config = (MotionFieldConfig)target;

        EditorGUILayout.Space();
        DrawImportSection(config);

        EditorGUILayout.Space();
        DrawDatabaseSection(config);

        EditorGUILayout.Space();
        DrawBoneWeightSection(config);

        EditorGUILayout.Space();
        DrawTrainingSection(config);

        EditorGUILayout.Space();
        DrawVisualizationSection(config);

        if (GUI.changed)
        {
            EditorUtility.SetDirty(config);
        }
    }

    private void DrawImportSection(MotionFieldConfig config)
    {
        _showImport = EditorGUILayout.Foldout(_showImport, "Import Settings From MotionMatchingData", true);
        if (!_showImport) return;

        EditorGUILayout.HelpBox(
            "One-time copy of the clips, hips forward vector, contact threshold and Mecanim map. " +
            "It does not create a link -- later edits to the source asset will not follow.",
            MessageType.Info);

        _importSource = (MotionMatchingData)EditorGUILayout.ObjectField(
            "Source", _importSource, typeof(MotionMatchingData), false);

        using (new EditorGUI.DisabledScope(_importSource == null))
        {
            if (GUILayout.Button("Copy Settings"))
            {
                Undo.RecordObject(config, "Import MotionField settings");
                config.animationClips = new System.Collections.Generic.List<AnnotatedAnimationClip>(
                    _importSource.animationClips);
                config.hipsForwardLocalVector = _importSource.hipsForwardLocalVector;
                config.contactVelocityThreshold = _importSource.contactVelocityThreshold;
                config.animationChannelToMecanim =
                    new System.Collections.Generic.List<MotionMatchingData.JointToMecanim>(
                        _importSource.animationChannelToMecanim);
                EditorUtility.SetDirty(config);
                Debug.Log($"[MotionField] Copied {config.animationClips.Count} clip(s) and " +
                          $"{config.animationChannelToMecanim.Count} bone mapping(s) from " +
                          $"'{_importSource.name}'.");
            }
        }
    }

    private void DrawDatabaseSection(MotionFieldConfig config)
    {
        EditorGUILayout.LabelField("Pose Database", EditorStyles.boldLabel);
        EditorGUILayout.LabelField("Output", ProjectRelative(config.GetAssetPath()));

        bool hasClips = config.animationClips != null && config.animationClips.Count > 0;
        if (!hasClips)
        {
            EditorGUILayout.HelpBox("Assign at least one animation clip.", MessageType.Warning);
        }
        else if (config.animationChannelToMecanim == null || config.animationChannelToMecanim.Count == 0)
        {
            EditorGUILayout.HelpBox(
                "No Mecanim bone mapping. Pose extraction locates the toes through it for foot " +
                "contact detection and will fail without it.", MessageType.Warning);
        }

        using (new EditorGUI.DisabledScope(!hasClips))
        {
            if (GUILayout.Button("Generate Pose Database", GUILayout.Height(24)))
            {
                GeneratePoseDatabase(config);
                InvalidateSkeleton(); // the joint list the bone weight rows are drawn from
            }
        }
    }

    /// <summary>
    /// Per-joint weights, drawn as the skeleton hierarchy.
    /// </summary>
    /// <remarks>
    /// The list on the asset is name-keyed and sparse, so a default list drawer would show a
    /// handful of anonymous elements in whatever order they were edited. Here every joint of the
    /// database gets a row, indented by its depth, whether or not it carries a stored weight.
    /// </remarks>
    private void DrawBoneWeightSection(MotionFieldConfig config)
    {
        _showBoneWeights = EditorGUILayout.Foldout(
            _showBoneWeights, "Bone Weights (similarity metric)", true);
        if (!_showBoneWeights) return;

        if (!TryReadSkeleton(config))
        {
            EditorGUILayout.HelpBox(
                "Generate the pose database first. The bone list is read from the generated " +
                ".mmskeleton, which is the skeleton the similarity metric is computed over.",
                MessageType.Warning);
            return;
        }

        EditorGUILayout.HelpBox(
            "Multiplies each joint's contribution to the k-NN distance, on top of the global " +
            "Pos Weight and Vel Weight. The metric sums over joints, so lowering one stops it " +
            "outvoting the joints you care about; 0 removes it from matching entirely.\n" +
            "This changes the metric, so retrain the value function afterwards -- until then " +
            "the stage rejects the stale one and falls back to greedy control.",
            MessageType.None);

        DrawBoneWeightRows(config);
        DrawBoneWeightFooter(config);
    }

    private void DrawBoneWeightRows(MotionFieldConfig config)
    {
        const float fieldWidth = 54f;
        List<Skeleton.Joint> joints = _skeleton.Joints;

        using (new EditorGUILayout.HorizontalScope())
        {
            EditorGUILayout.LabelField("Joint", EditorStyles.miniBoldLabel);
            EditorGUILayout.LabelField("Pos", EditorStyles.miniBoldLabel, GUILayout.Width(fieldWidth));
            EditorGUILayout.LabelField("Vel", EditorStyles.miniBoldLabel, GUILayout.Width(fieldWidth));
        }

        for (int i = 0; i < joints.Count; i++)
        {
            Skeleton.Joint joint = joints[i];
            MotionFieldConfig.BoneWeight weight = config.GetBoneWeight(joint.name);

            // Row 0 is the simulation bone. Forward kinematics pins it to the origin with identity
            // rotation before the feature is built, so its position and velocity are structurally
            // zero and no weight can change them. Shown for orientation, not editable.
            bool isRoot = i == 0;

            using (new EditorGUILayout.HorizontalScope())
            {
                GUILayout.Space(_jointDepths[i] * 12f);

                var label = new GUIContent(joint.name, isRoot
                    ? "The root is pinned to the origin when the metric is built, so its rows are " +
                      "always zero and its weight has no effect."
                    : $"{joint.type} (joint {i})");
                EditorGUILayout.LabelField(label,
                    weight.IsNeutral ? EditorStyles.label : EditorStyles.boldLabel);

                using (new EditorGUI.DisabledScope(isRoot))
                {
                    float position = EditorGUILayout.FloatField(
                        weight.position, GUILayout.Width(fieldWidth));
                    float velocity = EditorGUILayout.FloatField(
                        weight.velocity, GUILayout.Width(fieldWidth));

                    if (isRoot) continue;
                    if (Mathf.Approximately(position, weight.position) &&
                        Mathf.Approximately(velocity, weight.velocity)) continue;

                    Undo.RecordObject(config, "Edit bone weight");
                    config.SetBoneWeight(new MotionFieldConfig.BoneWeight(
                        joint.name, Mathf.Max(0f, position), Mathf.Max(0f, velocity)));
                    EditorUtility.SetDirty(config);
                }
            }
        }
    }

    private void DrawBoneWeightFooter(MotionFieldConfig config)
    {
        string[] weighted = config.boneWeights
            .Where(w => !w.IsNeutral)
            .Select(w => $"{w.name} ({w.position:0.##}/{w.velocity:0.##})")
            .ToArray();

        EditorGUILayout.LabelField(
            weighted.Length == 0 ? "All joints at 1 -- metric unchanged." : string.Join(", ", weighted),
            EditorStyles.wordWrappedMiniLabel);

        using (new EditorGUILayout.HorizontalScope())
        {
            using (new EditorGUI.DisabledScope(weighted.Length == 0))
            {
                if (GUILayout.Button("Reset All To 1"))
                {
                    Undo.RecordObject(config, "Reset bone weights");
                    config.boneWeights.Clear();
                    EditorUtility.SetDirty(config);
                }
            }

            if (GUILayout.Button("Reload Skeleton"))
            {
                InvalidateSkeleton();
            }
        }

        // Entries naming joints the current skeleton does not have. They are ignored at runtime
        // (Python logs them), but silently keeping them would make the summary above lie.
        var orphans = config.boneWeights
            .Where(w => _skeleton.Joints.All(j => j.name != w.name))
            .ToList();
        if (orphans.Count == 0) return;

        EditorGUILayout.HelpBox(
            $"{orphans.Count} weight(s) name joints this skeleton does not have and are ignored: " +
            string.Join(", ", orphans.Select(w => w.name)), MessageType.Warning);

        if (GUILayout.Button("Remove Stale Entries"))
        {
            Undo.RecordObject(config, "Remove stale bone weights");
            config.boneWeights.RemoveAll(w => _skeleton.Joints.All(j => j.name != w.name));
            EditorUtility.SetDirty(config);
        }
    }

    /// <summary>Read and cache the database skeleton, with each joint's depth for indentation.</summary>
    private bool TryReadSkeleton(MotionFieldConfig config)
    {
        if (_skeletonRead) return _skeleton != null;
        _skeletonRead = true;

        if (!config.TryGetDatabaseSkeleton(out Skeleton skeleton) || skeleton.Joints.Count == 0)
        {
            return false;
        }

        _skeleton = skeleton;
        _jointDepths = new int[skeleton.Joints.Count];

        for (int i = 0; i < skeleton.Joints.Count; i++)
        {
            int depth = 0;
            int parent = skeleton.Joints[i].parentIndex;
            // Bounded by the joint count so a cyclic parentIndex cannot hang the inspector.
            while (parent >= 0 && parent < skeleton.Joints.Count && depth < skeleton.Joints.Count)
            {
                depth++;
                parent = skeleton.Joints[parent].parentIndex;
            }

            _jointDepths[i] = depth;
        }

        return true;
    }

    private void DrawTrainingSection(MotionFieldConfig config)
    {
        EditorGUILayout.LabelField("Value Function", EditorStyles.boldLabel);

        string valuePath = config.GetValueFunctionPath();
        bool databaseExists = File.Exists(Path.Combine(config.GetAssetPath(), config.name + ".mmpose"));
        bool trained = File.Exists(valuePath);

        EditorGUILayout.LabelField("Trained file",
            trained
                ? $"{ProjectRelative(valuePath)}  ({new FileInfo(valuePath).Length / 1024} KB)"
                : "not trained yet");

        if (!databaseExists)
        {
            EditorGUILayout.HelpBox("Generate the pose database first.", MessageType.Warning);
        }

        if (EditorApplication.isPlayingOrWillChangePlaymode)
        {
            EditorGUILayout.HelpBox(
                "Training is disabled in play mode. It would hold the Python GIL for seconds at a " +
                "time and stall the running MotionFieldStage.", MessageType.Info);
        }

        using (new EditorGUI.DisabledScope(!databaseExists || EditorApplication.isPlayingOrWillChangePlaymode))
        {
            if (GUILayout.Button("Train Motion Field", GUILayout.Height(30)))
            {
                TrainMotionField(config, force: false);
            }

            if (GUILayout.Button("Retrain (ignore transition cache)"))
            {
                TrainMotionField(config, force: true);
            }
        }
    }

    private void DrawVisualizationSection(MotionFieldConfig config)
    {
        EditorGUILayout.LabelField("Debug Visualization", EditorStyles.boldLabel);

        string embeddingPath = config.GetEmbeddingPath();
        bool databaseExists = File.Exists(Path.Combine(config.GetAssetPath(), config.name + ".mmpose"));
        bool embedded = File.Exists(embeddingPath);

        EditorGUILayout.LabelField("Embedding",
            embedded
                ? $"{ProjectRelative(embeddingPath)}  ({new FileInfo(embeddingPath).Length / 1024} KB)"
                : "not computed yet");

        if (!databaseExists)
        {
            EditorGUILayout.HelpBox("Generate the pose database first.", MessageType.Warning);
        }

        EditorGUILayout.HelpBox(
            "Projects the motion field to 3D so MotionFieldVisualizer can draw it. Independent of " +
            "training -- the value function does not need it, and deleting it only turns the " +
            "visualizer off. Expect ~20 s: the first UMAP import pays a one-off numba warm-up.",
            MessageType.None);

        using (new EditorGUI.DisabledScope(!databaseExists || EditorApplication.isPlayingOrWillChangePlaymode))
        {
            if (GUILayout.Button("Compute UMAP Embedding", GUILayout.Height(30)))
            {
                ComputeEmbedding(config);
            }
        }
    }

    private static void GeneratePoseDatabase(MotionFieldConfig config)
    {
        try
        {
            EditorUtility.DisplayProgressBar("Motion Field", "Extracting poses...", 0.3f);
            config.ImportPoseSet();

            EditorUtility.DisplayProgressBar("Motion Field", "Writing database...", 0.7f);
            new PoseSerializer().Serialize(config.GetOrImportPoseSet(), config.GetAssetPath(), config.name);

            Debug.Log($"[MotionField] Wrote {config.GetOrImportPoseSet().NumberPoses} poses to " +
                      $"{ProjectRelative(config.GetAssetPath())}.");
        }
        catch (Exception e)
        {
            Debug.LogException(e);
        }
        finally
        {
            EditorUtility.ClearProgressBar();
            config.InvalidatePoseSet();
            AssetDatabase.Refresh();
        }
    }

    private static void TrainMotionField(MotionFieldConfig config, bool force)
    {
        // The interpreter runs in this process. A domain reload while it is mid-call takes the
        // editor down with it, so hold reloads off for the duration.
        EditorApplication.LockReloadAssemblies();
        try
        {
            PythonRuntime.EnsureInitialized(config.pythonDllPath, config.pythonVenvPath);

            // Marshalled into Python as a callable. Training is synchronous on the main thread, so
            // this fires on the main thread too and may touch the editor UI directly.
            Action<string, double> report = (stage, fraction) =>
                EditorUtility.DisplayProgressBar("Training Motion Field", stage, (float)fraction);

            using (Py.GIL())
            {
                PyObject trainer = PythonRuntime.Import("motion_field_trainer", reload: true);

                using var args = new PyTuple(new[]
                {
                    config.GetAssetPath().ToPython(),
                    config.name.ToPython(),
                    config.GetValueFunctionPath().ToPython(),
                });

                using var kwargs = new PyDict();
                kwargs["cache_path"] = config.GetTablesCachePath().ToPython();
                kwargs["k_neighbors"] = config.kNeighbors.ToPython();
                kwargs["theta_count"] = config.thetaCount.ToPython();
                kwargs["epochs"] = config.epochs.ToPython();
                kwargs["gamma"] = ((double)config.gamma).ToPython();
                kwargs["tug_ratio"] = ((double)config.tugRatio).ToPython();
                kwargs["pos_weight"] = ((double)config.posWeight).ToPython();
                kwargs["vel_weight"] = ((double)config.velWeight).ToPython();
                kwargs["bone_weights"] = MotionFieldBoneWeights.ToPython(config);
                kwargs["device"] = config.DeviceName.ToPython();
                kwargs["knn_chunk"] = config.knnChunk.ToPython();
                kwargs["state_chunk"] = config.stateChunk.ToPython();
                kwargs["force"] = force.ToPython();
                kwargs["progress"] = report.ToPython();

                using PyObject summary = trainer.InvokeMethod("train", args, kwargs);
                Debug.Log($"[MotionField] {summary}");
            }

            GC.KeepAlive(report);
        }
        catch (Exception e)
        {
            Debug.LogException(e);
        }
        finally
        {
            EditorUtility.ClearProgressBar();
            EditorApplication.UnlockReloadAssemblies();
            AssetDatabase.Refresh();
        }
    }

    /// <summary>
    /// Fit the UMAP projection of the database. Same execution model as
    /// <see cref="TrainMotionField"/>: in-process, synchronous, reloads locked out.
    /// </summary>
    private static void ComputeEmbedding(MotionFieldConfig config)
    {
        EditorApplication.LockReloadAssemblies();
        try
        {
            PythonRuntime.EnsureInitialized(config.pythonDllPath, config.pythonVenvPath);

            Action<string, double> report = (stage, fraction) =>
                EditorUtility.DisplayProgressBar("Embedding Motion Field", stage, (float)fraction);

            using (Py.GIL())
            {
                PyObject embedding = PythonRuntime.Import("motion_field_embedding", reload: true);

                using var args = new PyTuple(new[]
                {
                    config.GetAssetPath().ToPython(),
                    config.name.ToPython(),
                    config.GetEmbeddingPath().ToPython(),
                });

                using var kwargs = new PyDict();
                kwargs["n_components"] = config.umapComponents.ToPython();
                kwargs["n_neighbors"] = config.umapNeighbors.ToPython();
                kwargs["min_dist"] = ((double)config.umapMinDist).ToPython();
                kwargs["device"] = config.DeviceName.ToPython();
                kwargs["seed"] = config.umapSeed.ToPython();
                kwargs["knn_chunk"] = config.knnChunk.ToPython();
                kwargs["pos_weight"] = ((double)config.posWeight).ToPython();
                kwargs["vel_weight"] = ((double)config.velWeight).ToPython();
                kwargs["bone_weights"] = MotionFieldBoneWeights.ToPython(config);
                kwargs["progress"] = report.ToPython();

                using PyObject summary = embedding.InvokeMethod("compute_embedding", args, kwargs);
                Debug.Log($"[MotionField] {summary}");
            }

            GC.KeepAlive(report);
        }
        catch (Exception e)
        {
            Debug.LogException(e);
        }
        finally
        {
            EditorUtility.ClearProgressBar();
            EditorApplication.UnlockReloadAssemblies();
            AssetDatabase.Refresh();
        }
    }

    private static string ProjectRelative(string absolute)
    {
        string root = Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
        string full = Path.GetFullPath(absolute);
        return full.StartsWith(root, StringComparison.OrdinalIgnoreCase)
            ? full.Substring(root.Length).TrimStart(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar)
            : full;
    }
}
}