# About

This is an experimental, very much work-in-progress plugin to validate use of
the [KHR_rigid_bodies](https://github.com/eoineoineoin/glTF_Physics) glTF extension
for adding rigid body support to glTF files

# Todo

Materials

# Known issues

Godot Physics does not support mesh-mesh collisions; this plugin will create
meshes whenever the input file has requested them, even if it may result in a
collision pair which is unsupported by Godot.

Godot constraint spaces are parameterized on a single world-space transform
(rather than specifying a coordinate system in the space of each body), which
implies that the pivots of both bodies are aligned at scene load. When this is
not the case in the input file, we lose information about the pivot on body B.
