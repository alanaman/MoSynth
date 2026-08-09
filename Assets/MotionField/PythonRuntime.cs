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
    /// Pick up edits to the .py files without restarting Unity, by dropping the whole project
    /// module graph first -- see <see cref="InvalidateProjectModules"/>. Call it once before a
    /// group of related imports rather than on each one, so they all end up sharing one generation
    /// of the code.
    /// </param>
    public static dynamic Import(string moduleName, bool reload = false)
    {
        if (reload) InvalidateProjectModules();
        return Py.Import(moduleName);
    }

    /// <summary>
    /// Forget every already-imported module that lives under <see cref="ScriptsFolder"/>, so the
    /// next import re-reads them from disk. Returns how many were dropped.
    /// </summary>
    /// <remarks>
    /// This replaces a per-module <c>importlib.reload</c>, which only re-executes the one module it
    /// is handed. Its dependencies stay cached, so reloading <c>MotionField</c> after adding a
    /// symbol to <c>motion_field_io</c> re-runs the new import line against the old dependency and
    /// dies on ImportError -- editing a shared module was effectively impossible without restarting
    /// the editor.
    ///
    /// Dropping the graph wholesale also means a group of imports issued after one call sees a
    /// single consistent generation of the code. Reloading module by module does not: each reload
    /// rebinds only that module's classes, so two modules can end up holding different class
    /// objects for the same class.
    ///
    /// Live Python objects created before the call keep working -- they hold a reference to the
    /// class they were built from, which simply is no longer the one a fresh import returns. Call
    /// this only where everything downstream is about to be rebuilt.
    /// </remarks>
    public static int InvalidateProjectModules()
    {
        using PyModule scope = Py.CreateScope();
        scope.Set("root", ScriptsFolder.ToPython());
        scope.Exec(InvalidateScript);
        return scope.Get<int>("dropped");
    }

    // Kept as source rather than a .py file in ScriptsFolder: a module that purges the project's
    // modules should not be one of the modules it purges.
    private const string InvalidateScript = @"
import os
import sys

prefix = os.path.normcase(os.path.abspath(root)) + os.sep
stale = []
for name, module in list(sys.modules.items()):
    path = getattr(module, '__file__', None)
    if not path:
        continue
    if os.path.normcase(os.path.abspath(path)).startswith(prefix):
        stale.append(name)

for name in stale:
    del sys.modules[name]

dropped = len(stale)
";
}
}
