# -*- mode: python ; coding: utf-8 -*-


block_cipher = None


a = Analysis(
    ['simulator/main.py'],
    pathex=['simulator'],
    binaries=[],
    datas=[
        ('buttons', 'buttons'),
        ('assets', 'assets'),
        # Recursos del motor 3D del Braccio (mallas STL + preset DH). Viven en
        # simulator/assets y el motor3d los resuelve como <bundle>/assets/{stl,presets}.
        ('simulator/assets/stl', 'assets/stl'),
        ('simulator/assets/presets', 'assets/presets'),
        # Sketches de los retos (robots.py los abre como codes/challengeN) y el
        # tutorial PDF que enlaza la interfaz (drawing.py -> tutorials/Tutorial.pdf).
        ('codes', 'codes'),
        ('tutorials', 'tutorials'),
        ('robot_data.json', '.'),
        ('manual-usuario.pdf', '.')
    ],
    hiddenimports=[],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)
pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='simulador',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=True,
    icon='assets/simlogo.ico',
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
coll = COLLECT(
    exe,
    a.binaries,
    a.zipfiles,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='simulador',
)
