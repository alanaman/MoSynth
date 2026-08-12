using System;
using System.Collections.Generic;
using System.Reflection;
using UnityEditor;
using UnityEngine;

namespace AnimationTools.Editor
{
/// <summary>
/// Draws a <see cref="BoneReference"/> as a bone-name dropdown instead of a raw id field.
/// Resolves the skeleton to populate the dropdown from, in order, a sibling field named by a
/// <see cref="BoneFromAttribute"/> on the reference field, then an <see cref="ISkeletonProvider"/>
/// on the owning object.
/// </summary>
[CustomPropertyDrawer(typeof(BoneReference))]
public class BoneReferenceDrawer : PropertyDrawer
{
    private const string NoneOption = "(none)";
    private const int IndentSpacesPerDepth = 4;

    private struct CacheEntry
    {
        public int version;

        // options[0] is always "(none)"; options[i + 1] is the display name for skeleton.GetBone(i).
        public string[] options;
    }

    private static readonly Dictionary<int, CacheEntry> Caches = new();

    public override float GetPropertyHeight(SerializedProperty property, GUIContent label)
    {
        return EditorGUIUtility.singleLineHeight;
    }

    public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
    {
        var boneIdProp = property.FindPropertyRelative("boneId");
        var cachedNameProp = property.FindPropertyRelative("cachedName");

        var skeleton = ResolveSkeleton(property);

        EditorGUI.BeginProperty(position, label, property);

        if (skeleton == null)
        {
            DrawUnresolved(position, label, boneIdProp, cachedNameProp);
            EditorGUI.EndProperty();
            return;
        }

        DrawPopup(position, label, skeleton, boneIdProp, cachedNameProp);
        EditorGUI.EndProperty();
    }

    // --- Skeleton resolution ---------------------------------------------------------------

    private SkeletonAsset ResolveSkeleton(SerializedProperty property)
    {
        var boneFrom = fieldInfo?.GetCustomAttribute<BoneFromAttribute>();
        if (boneFrom != null)
        {
            var siblingPath = GetSiblingPath(property.propertyPath, boneFrom.SkeletonMemberName);
            var skeletonProp = property.serializedObject.FindProperty(siblingPath);
            if (skeletonProp != null && skeletonProp.objectReferenceValue is SkeletonAsset fromAttribute)
                return fromAttribute;
        }

        if (property.serializedObject.targetObject is ISkeletonProvider provider)
            return provider.PoseSkeleton;

        return null;
    }

    /// <summary>
    /// Maps a property path to a same-level sibling, stripping any trailing array index so a
    /// <c>[BoneFrom]</c> on a list element still finds a field on the element's own owner (e.g.
    /// <c>stages.Array.data[0].contactBones.Array.data[1]</c> -&gt; <c>stages.Array.data[0].skeleton</c>).
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

    // --- Drawing -----------------------------------------------------------------------------

    private static void DrawUnresolved(
        Rect position, GUIContent label, SerializedProperty boneIdProp, SerializedProperty cachedNameProp)
    {
        var display = boneIdProp.intValue > 0 ? cachedNameProp.stringValue : NoneOption;
        var content = new GUIContent(display,
            "No SkeletonAsset found — add [BoneFrom] or implement ISkeletonProvider.");

        using (new EditorGUI.DisabledScope(true))
        {
            EditorGUI.Popup(position, label, 0, new[] { content });
        }
    }

    private static void DrawPopup(
        Rect position, GUIContent label, SkeletonAsset skeleton,
        SerializedProperty boneIdProp, SerializedProperty cachedNameProp)
    {
        var cache = GetOrBuildCache(skeleton);

        var boneId = boneIdProp.intValue;
        var skeletonIndex = boneId > 0 ? skeleton.IndexOfId(boneId) : -1;
        var missing = boneId > 0 && skeletonIndex < 0;

        string[] options;
        int optionOffset;
        int currentIndex;

        if (missing)
        {
            options = new string[cache.options.Length + 1];
            options[0] = $"{cachedNameProp.stringValue} (missing)";
            Array.Copy(cache.options, 0, options, 1, cache.options.Length);
            optionOffset = 1;
            currentIndex = 0;
        }
        else
        {
            options = cache.options;
            optionOffset = 0;
            currentIndex = boneId > 0 ? skeletonIndex + 1 : 0;
        }

        var priorColor = GUI.color;
        if (missing) GUI.color = Color.red;

        EditorGUI.BeginChangeCheck();
        var newIndex = EditorGUI.Popup(position, label.text, currentIndex, options);
        var changed = EditorGUI.EndChangeCheck();

        GUI.color = priorColor;

        if (!changed) return;

        var cacheIndex = newIndex - optionOffset;
        if (cacheIndex < 0) return; // the "<cachedName> (missing)" sentinel was re-selected; leave untouched

        if (cacheIndex == 0)
        {
            boneIdProp.intValue = 0;
            cachedNameProp.stringValue = "";
        }
        else
        {
            var bone = skeleton.GetBone(cacheIndex - 1);
            boneIdProp.intValue = bone.id;
            cachedNameProp.stringValue = bone.name;
        }
    }

    private static CacheEntry GetOrBuildCache(SkeletonAsset skeleton)
    {
        var key = skeleton.GetEntityId().GetHashCode();
        if (Caches.TryGetValue(key, out var cached) && cached.version == skeleton.Version)
            return cached;

        var count = skeleton.BoneCount;
        var depths = new int[count];
        var nameCounts = new Dictionary<string, int>();

        for (var i = 0; i < count; i++)
        {
            var bone = skeleton.GetBone(i);
            nameCounts.TryGetValue(bone.name, out var n);
            nameCounts[bone.name] = n + 1;

            var depth = 0;
            var parent = bone.parentIndex;
            var guard = 0;
            while (parent >= 0 && guard++ < count)
            {
                depth++;
                parent = skeleton.GetBone(parent).parentIndex;
            }

            depths[i] = depth;
        }

        var options = new string[count + 1];
        options[0] = NoneOption;
        for (var i = 0; i < count; i++)
        {
            var bone = skeleton.GetBone(i);
            var indent = new string(' ', depths[i] * IndentSpacesPerDepth);
            var displayName = nameCounts[bone.name] > 1 ? $"{bone.name} ({bone.id})" : bone.name;
            options[i + 1] = indent + displayName;
        }

        var entry = new CacheEntry { version = skeleton.Version, options = options };
        Caches[key] = entry;
        return entry;
    }
}
}
