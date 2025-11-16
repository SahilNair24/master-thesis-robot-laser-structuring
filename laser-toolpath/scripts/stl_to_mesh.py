#!/usr/bin/env python3
import trimesh
import pickle
import argparse
from pathlib import Path
from shape_msgs.msg import Mesh, MeshTriangle
from geometry_msgs.msg import Point

def stl_to_shape_msg(path):
    """Convert STL file to ROS Mesh message."""
    mesh = trimesh.load_mesh(path)
    shape_msg = Mesh()
    
    # Add triangles (faces)
    for face in mesh.faces:
        shape_msg.triangles.append(MeshTriangle(vertex_indices=face.tolist()))
    
    # Add vertices
    for vert in mesh.vertices:
        shape_msg.vertices.append(Point(x=vert[0], y=vert[1], z=vert[2]))
    
    return shape_msg

def save_mesh_msg(mesh_msg, output_path):
    """Serialize and save ROS Mesh message to file."""
    with open(output_path, 'wb') as f:
        pickle.dump(mesh_msg, f)
    print(f"Saved Mesh message to: {output_path}")

def main():
    parser = argparse.ArgumentParser(description='Convert STL to serialized ROS Mesh')
    # parser.add_argument('input_stl', help='Path to input STL file', default='/home/sahilsnair/ros2_ws/src/robotpath/meshes/FemaleHead.stl')
    parser.add_argument('--output', '-o', default='mesh_msg.pkl', 
                       help='Output file path (default: mesh_msg.pkl)')
    args = parser.parse_args()

    # Convert STL to ROS Mesh message
    mesh_msg = stl_to_shape_msg('/home/sahilsnair/ros2_ws/src/robotpath/meshes/FemaleHead.stl')
    
    # Save to file
    save_mesh_msg(mesh_msg, args.output)

if __name__ == "__main__":
    main()