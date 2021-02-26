import bpy
import glob
import os


importDir = "./visual/"
exportDir = "../collision/" #relative to import (or absolute)

print(importDir)
print(exportDir)

# change to importDir
os.chdir(importDir)

# get the current path and make a new folder for the exported meshes
if not os.path.exists(exportDir):
    os.makedirs(exportDir)

for files in glob.glob("*.stl"):
    print( files )
    #delete any objects
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)

    #import file
    bpy.ops.import_mesh.stl(filepath=files, filter_glob="*.stl")

    #select object
    bpy.ops.object.select_all(action='SELECT')

    #edit mode
    bpy.ops.object.mode_set(mode='EDIT', toggle=False)

    #select mesh
    bpy.ops.mesh.select_all(action='SELECT')

    #create convex hull
    bpy.ops.mesh.convex_hull()

    #decimate mesh
    bpy.ops.object.mode_set(mode='OBJECT', toggle=False)
    bpy.ops.object.modifier_add(type='DECIMATE')
    bpy.context.object.modifiers["Decimate"].angle_limit = 0.10472
    bpy.ops.object.modifier_apply(apply_as='DATA', modifier="Decimate")

    # export object with its name as file name
    fPath = str((exportDir + files))
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.export_mesh.stl(filepath=fPath)
    print ("generated " + fPath)
