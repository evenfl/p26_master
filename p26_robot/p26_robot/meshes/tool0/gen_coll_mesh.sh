#!/bin/bash

# run in mesh folder. extracts stl files from ./visual/ and generates ./collision
blender -b -P gen_coll_mesh.py
