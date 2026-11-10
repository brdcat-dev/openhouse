#!/usr/bin/env python3
"""Strip and package the cross-built Windows DLL and standalone apps.
Reject unexpected third-party DLL dependencies rather than ship a broken bundle.
"""
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys

repo = Path(__file__).resolve().parent.parent
build = Path(sys.argv[1]).resolve()
prefix = Path(sys.argv[2]).resolve()
out = repo / 'dist/openhouse-windows-x64'
shutil.rmtree(out, ignore_errors=True)
(out / 'apps').mkdir(parents=True)
files = {build / 'openvr/lighthouse/bin/win64/driver_lighthouse.dll': out / 'driver_lighthouse.dll',
         build / 'openhouse-installer.exe': out / 'openhouse-installer.exe'}
files.update({build / name: out / 'apps' / name for name in ['liblh.dll', 'openhouse-viewer.exe', 'openhouse-compare.exe', 'lhviz.exe', 'lhtool.exe', 'lhcapture.exe', 'openhouse-check.exe']})
bridge = build / 'bluetooth/nrfserlib.dll'
if bridge.is_file():
    (out / 'bluetooth').mkdir()
    files[bridge] = out / 'bluetooth/nrfserlib.dll'
system = {'advapi32.dll', 'winhttp.dll', 'kernel32.dll', 'shlwapi.dll', 'user32.dll', 'gdi32.dll',
          'shell32.dll', 'winmm.dll', 'ole32.dll', 'comdlg32.dll', 'msvcrt.dll',
          'ucrtbase.dll', 'ntdll.dll', 'imm32.dll', 'version.dll', 'oleaut32.dll', 'setupapi.dll', 'opengl32.dll', 'hid.dll', 'ws2_32.dll', 'bcrypt.dll'}
artifacts = []
for source, target in files.items():
    if not source.is_file():
        sys.exit(f'Missing build artifact: {source}')
    if source == bridge:
        shutil.copy2(source, target)
    else:
        subprocess.run(['x86_64-w64-mingw32-strip', '--strip-unneeded', '-o', str(target), str(source)], check=True)
    pe = subprocess.check_output(['x86_64-w64-mingw32-objdump', '-p', str(target)], text=True)
    imports = re.findall(r'DLL Name:\s*(\S+)', pe)
    for name in imports:
        if name.lower() not in system and not name.lower().startswith('api-ms-win-') and not (target.parent / name).is_file() and name.lower() != 'liblh.dll':
            sys.exit(f'Unbundled Windows dependency in {target}: {name}')
        if name.lower() == 'liblh.dll' and target.parent != out / 'apps':
            sys.exit('Driver must not depend on liblh.dll')
    artifacts.append({'file': str(target.relative_to(out)), 'imports': imports,
                      'sha256': hashlib.sha256(target.read_bytes()).hexdigest()})
licenses = out / 'licenses'
licenses.mkdir(exist_ok=True)
for copyright in (prefix / 'share').glob('*/copyright'):
    shutil.copy2(copyright, licenses / (copyright.parent.name + '.txt'))
shutil.copy2(repo / 'third_party/imgui/LICENSE.txt', licenses / 'imgui.txt')
shutil.copy2(repo / 'LICENSE', out / 'LICENSE.txt')
if Path('/usr/share/licenses/openvr').is_dir():
    shutil.copytree('/usr/share/licenses/openvr', licenses / 'openvr', dirs_exist_ok=True)
for package in ['mingw-w64-crt', 'mingw-w64-headers', 'mingw-w64-winpthreads', 'gcc']:
    notice = Path('/usr/share/licenses') / package
    if notice.is_dir():
        shutil.copytree(notice, licenses / package, dirs_exist_ok=True)
gpl = Path('/usr/share/licenses/spdx/GPL-3.0-only.txt')
if gpl.is_file():
    shutil.copy2(gpl, licenses / 'GCC-GPL-3.0.txt')
shutil.copy2(repo / 'scripts/windows-bluetooth.ps1', out / 'openhouse-bluetooth.ps1')
shutil.copy2(repo / 'packaging/settingsschema.vrsettings', out / 'settingsschema.vrsettings')
(out / 'BUILD.json').write_text(json.dumps({
    'source_commit': subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip(),
    'source_dirty': bool(subprocess.check_output(['git', '-C', str(repo), 'diff', '--name-only'], text=True).strip()),
    'compiler': subprocess.check_output(['x86_64-w64-mingw32-g++', '-dumpfullversion'], text=True).strip(),
    'vcpkg_baseline': json.loads((repo / 'vcpkg.json').read_text())['builtin-baseline'],
    'artifacts': artifacts,
}, indent=2) + '\n')
archive = shutil.make_archive(str(repo / 'dist/openhouse-windows-x64'), 'zip', out.parent, out.name)
print(f'Packaged {archive}')
for item in artifacts:
    print(f"  {item['file']}: {(out / item['file']).stat().st_size / 1048576:.1f} MiB")
