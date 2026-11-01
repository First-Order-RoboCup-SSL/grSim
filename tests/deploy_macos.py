#!/usr/bin/env python3
"""Complete and audit macdeployqt's transitive library closure before signing."""
import hashlib
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import sys


def output(*args):
    return subprocess.check_output(args, text=True)


def macho_files(directory):
    return [p for p in directory.rglob("*")
            if p.is_file() and not p.is_symlink() and "Mach-O" in output("file", "-b", str(p))]


def metadata(path):
    commands = output("otool", "-arch", "arm64", "-l", str(path))
    rpaths = []
    for block in commands.split("Load command "):
        if "LC_RPATH" in block:
            match = re.search(r"(?m)^\s+path (.*?) \(offset", block)
            if match:
                rpaths.append(match.group(1))
    identifiers = {s.strip() for s in output("otool", "-arch", "arm64", "-D", str(path)).splitlines()[1:]}
    dependencies = [s.strip().split(" (", 1)[0]
                    for s in output("otool", "-arch", "arm64", "-L", str(path)).splitlines()[1:]]
    return commands, rpaths, identifiers, [d for d in dependencies if d not in identifiers]


def deploy(app, brew_prefix, qt_prefix):
    app = app.resolve()
    frameworks = app/"Contents"/"Frameworks"
    executable = app/"Contents"/"MacOS"
    sources = {}
    pending = macho_files(app)
    known = set(pending)

    def inside(path):
        return path.resolve().is_relative_to(app)

    def system(path):
        return str(path).startswith(("/System/Library/", "/usr/lib/"))

    def expand(value, loader):
        if value.startswith("@loader_path"):
            return Path(str(loader.parent)+value[len("@loader_path"):])
        if value.startswith("@executable_path"):
            return Path(str(executable)+value[len("@executable_path"):])
        return Path(value)

    def original(path, rpaths, identifiers):
        candidates = [Path(s) for s in identifiers if s.startswith("/")]
        if path.is_relative_to(frameworks):
            candidates.append(qt_prefix/"lib"/path.relative_to(frameworks))
        if path.is_relative_to(app/"Contents"/"PlugIns"):
            candidates.append(qt_prefix/"plugins"/path.relative_to(app/"Contents"/"PlugIns"))
        candidates.append(brew_prefix/"lib"/path.name)
        candidates.extend(expand(r, path)/path.name for r in rpaths)
        return next((p.resolve() for p in candidates if p.is_file() and not inside(p)), path)

    def copy_library(source, dependency):
        source = source.resolve()
        if ".framework/" in str(source):
            framework = Path(str(source).split(".framework/", 1)[0]+".framework")
            destination = frameworks/framework.name
            relative = source.relative_to(framework)
            target = destination/relative
            if not destination.exists():
                shutil.copytree(framework, destination, symlinks=True)
                for copied in macho_files(destination):
                    sources[copied] = framework/copied.relative_to(destination)
                    if copied not in known:
                        pending.append(copied)
                        known.add(copied)
            elif not target.is_file():
                raise RuntimeError(f"Conflicting framework versions: {framework}")
            return target, "@rpath/"+str(target.relative_to(frameworks))
        target = frameworks/Path(dependency).name
        if target.exists():
            prior = sources.get(target, target)
            if prior.resolve() != source and hashlib.sha256(prior.read_bytes()).digest() != hashlib.sha256(source.read_bytes()).digest():
                raise RuntimeError(f"Conflicting library basename: {target.name}: {prior} vs {source}")
        else:
            shutil.copy2(source, target)
            sources[target] = source
            pending.append(target)
            known.add(target)
            print(f"Copied transitive dependency: {source} -> {target}", flush=True)
        return target, "@rpath/"+target.name

    # Resolve against original source locations/runpaths before deleting them.
    for path in pending:
        path.chmod(path.stat().st_mode | stat.S_IWUSR)
        _, rpaths, identifiers, dependencies = metadata(path)
        source = sources.setdefault(path, original(path, rpaths, identifiers))
        source_rpaths = metadata(source)[1] if source != path else rpaths
        for dependency in dependencies:
            if system(dependency):
                continue
            if dependency.startswith("@rpath/"):
                relative = dependency[len("@rpath/"):]
                candidates = [frameworks/relative]
                candidates += [expand(r, source)/relative for r in source_rpaths]
                candidates += [source.parent/relative, brew_prefix/"lib"/relative, qt_prefix/"lib"/relative]
            else:
                candidates = [expand(dependency, path), expand(dependency, source)]
            resolved = next((p for p in candidates if p.is_file()), None)
            if resolved is None:
                raise RuntimeError(f"Unresolved runtime dependency in {path}: {dependency}; source={source}")
            if not inside(resolved):
                resolved, replacement = copy_library(resolved, dependency)
                if dependency != replacement:
                    subprocess.check_call(["install_name_tool", "-change", dependency, replacement, str(path)])

    # Normalize identifiers and runpaths only after the closure is complete.
    for path in pending:
        commands, rpaths, identifiers, _ = metadata(path)
        if "arm64" not in output("lipo", "-archs", str(path)).split():
            raise RuntimeError(f"Missing arm64 slice: {path}")
        for block in commands.split("Load command "):
            if "LC_BUILD_VERSION" in block or "LC_VERSION_MIN_MACOSX" in block:
                match = re.search(r"(?m)^\s+(?:minos|version)\s+(\d+(?:\.\d+)*)", block)
                if match and (tuple(map(int, match.group(1).split(".")))+(0, 0))[:2] > (15, 0):
                    raise RuntimeError(f"Requires macOS newer than 15.0: {path}: {match.group(1)}")
        for identifier in identifiers:
            if identifier.startswith("/"):
                replacement = "@rpath/"+str(path.relative_to(frameworks)) if path.is_relative_to(frameworks) else "@rpath/"+path.name
                subprocess.check_call(["install_name_tool", "-id", replacement, str(path)])
        for rpath in rpaths:
            expanded = expand(rpath, path)
            if not inside(expanded) and not system(expanded):
                subprocess.check_call(["install_name_tool", "-delete_rpath", rpath, str(path)])
        if "@executable_path/../Frameworks" not in rpaths:
            subprocess.check_call(["install_name_tool", "-add_rpath", "@executable_path/../Frameworks", str(path)])

    for path in pending:
        _, rpaths, _, dependencies = metadata(path)
        for rpath in rpaths:
            if not inside(expand(rpath, path)) and not system(expand(rpath, path)):
                raise RuntimeError(f"External runtime search path: {path}: {rpath}")
        for dependency in dependencies:
            if system(dependency):
                continue
            resolved = (frameworks/dependency[len("@rpath/"):]) if dependency.startswith("@rpath/") else expand(dependency, path)
            if not resolved.is_file() or not inside(resolved):
                raise RuntimeError(f"Dependency escapes the bundle: {path}: {dependency}")
    print("PASS relocated bundle dependency closure and audit", flush=True)


if __name__ == "__main__":
    deploy(Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3]))
