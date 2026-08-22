using System;
using System.Collections.Generic;
using System.Reflection;
using UnityEditor;
using UnityEngine;

namespace AnimationTools.Editor
{
/// <summary>
/// Draws a <see cref="BoneTransform"/> as a bone-name dropdown over the transforms of a resolved
/// rig, instead of raw Transform/string fields. Resolves the rig root to populate the dropdown
/// from, in order, a sibling field named by a <see cref="BoneFromAttribute"/> on the reference
/// field, then an <see cref="ISkeletonProvider"/> on the owning object.
/// </summary>
[CustomPropertyDrawer(typeof(BoneTransform))]
public class BoneTransformDrawer : PropertyDrawer
{
    private const string NoneOption = "(none)";
    private const int IndentSpacesPerDepth = 4;

    public override float GetPropertyHeight(SerializedProperty property, GUIContent label)
    {
        return EditorGUIUtility.singleLineHeight;
    }

    public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
    {
        var rigRoot = ResolveRigRoot(property);

        EditorGUI.BeginProperty(position, label, property);
        DrawCore(position, label, property, rigRoot);
        EditorGUI.EndProperty();
    }

    /// <summary>
    /// Draws a <see cref="BoneTransform"/> popup for a caller that already knows the rig root
    /// (e.g. a custom editor whose asset resolves the rig from its own clips), bypassing
    /// [BoneFrom]/<see cref="ISkeletonProvider"/> resolution. <paramref name="rigRoot"/> may be
    /// null, which draws the same disabled fallback as an unresolved rig in <see cref="OnGUI"/>.
    /// </summary>
    public static void DrawLayout(GUIContent label, SerializedProperty boneTransformProperty, Transform rigRoot)
    {
        var position = EditorGUILayout.GetControlRect(true, EditorGUIUtility.singleLineHeight);

        EditorGUI.BeginProperty(position, label, boneTransformProperty);
        DrawCore(position, label, boneTransformProperty, rigRoot);
        EditorGUI.EndProperty();
    }

    // --- Rig resolution ------------------------------------------------------------------------

    private Transform ResolveRigRoot(SerializedProperty property)
    {
        var boneFrom = fieldInfo?.GetCustomAttribute<BoneFromAttribute>();
        if (boneFrom != null)
        {
            var rigRoot = ResolveFromSibling(property, boneFrom.SkeletonMemberName);
            if (rigRoot != null) return rigRoot;
        }

        if (property.serializedObject.targetObject is ISkeletonProvider provider)
            return provider.CharacterRig?.Root;

        return null;
    }

    private static Transform ResolveFromSibling(SerializedProperty property, string siblingName)
    {
        var siblingPath = GetSiblingPath(property.propertyPath, siblingName);

        // Sibling is a SkeletonRoot: its own "root" field holds the rig Transform directly.
        var rootProp = property.serializedObject.FindProperty(siblingPath + ".root");
        if (rootProp != null && rootProp.objectReferenceValue is Transform skeletonRootTransform)
            return skeletonRootTransform;

        // Sibling is an object reference to an AnnotatedAnimationClip or SkeletonAnimation asset.
        var siblingProp = property.serializedObject.FindProperty(siblingPath);
        return siblingProp != null ? RigRootFromAsset(siblingProp.objectReferenceValue) : null;
    }

    private static Transform RigRootFromAsset(UnityEngine.Object obj)
    {
        if (obj is SkeletonAnimation animation) return animation.Rig != null ? animation.Rig.transform : null;

        // The sibling may be the rig GameObject itself rather than an asset that references one.
        return obj is GameObject rig ? rig.transform : null;
    }

    /// <summary>
    /// Maps a property path to a same-level sibling, stripping any trailing array index so a
    /// <c>[BoneFrom]</c> on a list element still finds a field on the element's own owner (e.g.
    /// <c>features.Array.data[0].bone</c> -&gt; <c>features.Array.data[0].animationClips</c>).
    /// </summary>
    private static string GetSiblingPath(string propertyPath, string siblingName)
    {
        var path = propertyPath;
        if (path.EndsWith("]"))
        {
            var arrayIdx = path.LastIndexOf(".Array.data[", StringComparison.Ordinal);
            if (arrayIdx >= 0) path = path.Substring(0, arrayIdx);
        }

        var lastDot = path.LastIndexOf('.');
        return lastDot < 0 ? siblingName : path.Substring(0, lastDot + 1) + siblingName;
    }

    // --- Drawing ---------------------------------------------------------------------------------

    private static void DrawCore(Rect position, GUIContent label, SerializedProperty property, Transform rigRoot)
    {
        var boneProp = property.FindPropertyRelative("bone");
        var boneNameProp = property.FindPropertyRelative("boneName");

        if (rigRoot == null)
        {
            DrawUnresolved(position, label, boneNameProp);
            return;
        }

        var transforms = new List<Transform>();
        var depths = new List<int>();
        CollectTransformsDfs(rigRoot, 0, transforms, depths);

        DrawPopup(position, label, transforms, depths, boneProp, boneNameProp);
    }

    private static void CollectTransformsDfs(Transform transform, int depth, List<Transform> transforms, List<int> depths)
    {
        transforms.Add(transform);
        depths.Add(depth);
        for (var i = 0; i < transform.childCount; i++)
        {
            CollectTransformsDfs(transform.GetChild(i), depth + 1, transforms, depths);
        }
    }

    private static void DrawUnresolved(Rect position, GUIContent label, SerializedProperty boneNameProp)
    {
        var display = !string.IsNullOrEmpty(boneNameProp.stringValue) ? boneNameProp.stringValue : NoneOption;
        var content = new GUIContent(display,
            "No rig found — add [BoneFrom] or implement ISkeletonProvider.");

        using (new EditorGUI.DisabledScope(true))
        {
            EditorGUI.Popup(position, label, 0, new[] { content });
        }
    }

    private static void DrawPopup(
        Rect position, GUIContent label, List<Transform> transforms, List<int> depths,
        SerializedProperty boneProp, SerializedProperty boneNameProp)
    {
        var options = BuildOptions(transforms, depths);

        var boneTransform = boneProp.objectReferenceValue as Transform;
        var boneName = boneNameProp.stringValue;

        var resolvedIndex = boneTransform != null ? transforms.IndexOf(boneTransform) : -1;
        var missing = boneTransform == null && !string.IsNullOrEmpty(boneName) && !ContainsName(transforms, boneName);

        string[] displayOptions;
        int optionOffset;
        int currentIndex;

        if (missing)
        {
            displayOptions = new string[options.Length + 1];
            displayOptions[0] = $"(missing: {boneName})";
            Array.Copy(options, 0, displayOptions, 1, options.Length);
            optionOffset = 1;
            currentIndex = 0;
        }
        else
        {
            displayOptions = options;
            optionOffset = 0;
            currentIndex = resolvedIndex >= 0 ? resolvedIndex + 1 : 0;
        }

        var priorColor = GUI.color;
        if (missing) GUI.color = Color.red;

        EditorGUI.BeginChangeCheck();
        var newIndex = EditorGUI.Popup(position, label.text, currentIndex, displayOptions);
        var changed = EditorGUI.EndChangeCheck();

        GUI.color = priorColor;

        if (!changed) return;

        var optionIndex = newIndex - optionOffset;
        if (optionIndex < 0) return; // the "(missing: <name>)" sentinel was re-selected; leave untouched

        if (optionIndex == 0)
        {
            boneProp.objectReferenceValue = null;
            boneNameProp.stringValue = "";
        }
        else
        {
            var transform = transforms[optionIndex - 1];
            boneProp.objectReferenceValue = transform;
            boneNameProp.stringValue = transform.name;
        }
    }

    private static bool ContainsName(List<Transform> transforms, string name)
    {
        foreach (var t in transforms)
        {
            if (t.name == name) return true;
        }

        return false;
    }

    private static string[] BuildOptions(List<Transform> transforms, List<int> depths)
    {
        var nameCounts = new Dictionary<string, int>();
        foreach (var t in transforms)
        {
            nameCounts.TryGetValue(t.name, out var count);
            nameCounts[t.name] = count + 1;
        }

        var seenCounts = new Dictionary<string, int>();
        var options = new string[transforms.Count + 1];
        options[0] = NoneOption;

        for (var i = 0; i < transforms.Count; i++)
        {
            var t = transforms[i];
            var indent = new string(' ', depths[i] * IndentSpacesPerDepth);

            string displayName;
            if (nameCounts[t.name] > 1)
            {
                seenCounts.TryGetValue(t.name, out var seen);
                seen += 1;
                seenCounts[t.name] = seen;
                displayName = $"{t.name} ({seen})";
            }
            else
            {
                displayName = t.name;
            }

            options[i + 1] = indent + displayName;
        }

        return options;
    }
}
}
