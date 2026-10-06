# Joint 5 visual source

`joint5.obj` is the working joint 5 mesh from the previous
`mujoco_mycobot` project. The original assembly-style COLLADA file was skipped
by the Isaac Sim URDF importer, leaving the joint 5 visual prim empty.

`joint5_obj.usd` is generated from this OBJ with Isaac Sim's asset converter
and referenced by `../mycobot.usd` at the existing joint 5 visual transform.
