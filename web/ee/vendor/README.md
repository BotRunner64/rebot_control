# Three.js assets

Pinned version: **three 0.180.0**, MIT license (see `LICENSE.three.txt`).
Source package: https://registry.npmjs.org/three/-/three-0.180.0.tgz

Files are copied unmodified from the official npm package:

- `build/three.module.min.js` → `three.module.js`
- `build/three.core.min.js` → `three.core.min.js`
- `examples/jsm/controls/OrbitControls.js` → `OrbitControls.js`
- `examples/jsm/controls/TransformControls.js` → `TransformControls.js`

The viewer's import map resolves `three` to the local module. These files are
served locally so starting the viewer does not require npm, Node.js, or a CDN.
