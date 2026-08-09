using System;
using System.IO;
using Python.Runtime;
using UnityEngine;

namespace MotionField
{
/// <summary>
/// Shared CPython bootstrap for everything on the MotionField side.
///
/// The interpreter is process-wide and can only be configured once, so both the runtime stage and
/// the editor trainer have to come through here rather than each calling
/// <see cref="PythonEngine.Initialize()"/> with their own paths.
/// </summary>
public static class PythonRuntime
{
    private static bool _pythonDllAssigned;
    private static string _scriptsFolder;

    /// <summary>The repository's Python folder, which is where our modules are imported from.</summary>
    public static string ScriptsFolder =>
        _scriptsFolder ??= Path.GetFullPath(Path.Combine(Application.dataPath, "..", "Python"));

    /// <summary>
    /// Start CPython if it is not already running and make the project's Python modules importable.
    /// Safe to call repeatedly.
    /// </summary>
    /// <param name="pythonDllPath">
    /// Full path to the CPython shared library. Only meaningful on the very first call of the
    /// process -- pythonnet throws if it is changed after the interpreter starts. Empty falls back
    /// to the PYTHONNET_PYDLL environment variable.
    /// </param>
    /// <param name="venvPath">Virtual environment supplying numpy / scipy / torch. May be empty.</param>
    public static void EnsureInitialized(string pythonDllPath, string venvPath)
    {
        if (!PythonEngine.IsInitialized)
        {
            if (!_pythonDllAssigned && !string.IsNullOrWhiteSpace(pythonDllPath))
            {
                if (!File.Exists(pythonDllPath))
                {
                    throw new FileNotFoundException(
                        $"Python DLL not found at '{pythonDllPath}'. Set it on the MotionFieldConfig.",
                        pythonDllPath);
                }

                Runtime.PythonDLL = pythonDllPath;
                _pythonDllAssigned = true;
            }

            PythonEngine.Initialize();
        }

        using (Py.GIL())
        {
            if (!string.IsNullOrWhiteSpace(venvPath))
            {
                string sitePackages = Path.Combine(venvPath, "Lib", "site-packages");
                if (!Directory.Exists(sitePackages))
                {
                    throw new DirectoryNotFoundException(
                        $"No site-packages under '{venvPath}'. Set the venv path on the MotionFieldConfig.");
                }

                dynamic site = Py.Import("site");
                site.addsitedir(sitePackages);
            }

            dynamic sys = Py.Import("sys");
            AppendToPath(sys, ScriptsFolder);
        }
    }

    private static void AppendToPath(dynamic sys, string folder)
    {
        foreach (dynamic entry in sys.path)
        {
            if (string.Equals(entry.ToString(), folder, StringComparison.OrdinalIgnoreCase)) return;
        }

        sys.path.append(folder);
    }

    /// <summary>
    /// Import a module from the project's Python folder.
    /// </summary>
    /// <param name="moduleName"></param>
    /// <param name="reload">
    /// Re-execute the module so edits to the .py file take effect without restarting Unity. Handy
    /// while iterating; it also discards any module-level state the previous version held.
    /// </param>
    public static dynamic Import(string moduleName, bool reload = false)
    {
        dynamic module = Py.Import(moduleName);
        if (!reload) return module;

        dynamic importlib = Py.Import("importlib");
        return importlib.reload(module);
    }
}
}
