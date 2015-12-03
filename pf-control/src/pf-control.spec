# -*- mode: python -*-

block_cipher = None


a = Analysis(['pf-control.py'],
             pathex=['/Users/Arun/Documents/College/Classes/EEL4665C - IMDL/Pathfinder/pf-control'],
             binaries=None,
             datas=None,
             hiddenimports=[],
             hookspath=None,
             runtime_hooks=None,
             excludes=None,
             win_no_prefer_redirects=None,
             win_private_assemblies=None,
             cipher=block_cipher)
pyz = PYZ(a.pure, a.zipped_data,
             cipher=block_cipher)
exe = EXE(pyz,
          a.scripts,
          exclude_binaries=True,
          name='pf-control',
          debug=False,
          strip=None,
          upx=True,
          console=False , icon='pf-icon.icns')
coll = COLLECT(exe,
               a.binaries,
               a.zipfiles,
               a.datas,
               strip=None,
               upx=True,
               name='pf-control')
app = BUNDLE(coll,
             name='pf-control.app',
             icon='pf-icon.icns',
             bundle_identifier=None)
