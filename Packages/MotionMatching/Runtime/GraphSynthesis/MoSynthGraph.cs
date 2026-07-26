using System;
#if UNITY_EDITOR
using UnityEditor;
#endif
using Unity.GraphToolkit;
using UnityEngine;

namespace MotionMatching
{
    [Graph(AssetExtension)]
    [Serializable]
    public class MoSynthGraph : Graph
    {
        public const string AssetExtension = "mosynthgraph";

#if UNITY_EDITOR
        [MenuItem("Assets/Create/Motion Matching/MoSynth Graph", false)]
        public static void CreateAssetFile()
        {
            Unity.GraphToolkit.Editor.GraphDatabase.PromptInProjectBrowserToCreateNewAsset<MoSynthGraph>();
        }
#endif
    }
}
