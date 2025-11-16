# THIS IS A BOUNDING BOX IMPLEMENTATION FOR THE CAD SEGMENTATION

def create_oriented_bbox(mesh):
    """Create oriented bounding box using the proper method"""
    # Get the oriented bounding box - this is a property, not a method!
    obb = mesh.oriented_bounding_box()
    
    # For wireframe visualization, extract the edges
    return obb.extract_all_edges()

unique_clusters = np.unique(clusters)
# In your cluster processing loop:
for cluster_id in unique_clusters:
    cluster_cells = mesh.threshold(value=cluster_id, scalars='Cluster')

    
    
    if cluster_cells.n_cells == 0:  # Skip empty clusters
        continue
        
    cluster_surface = cluster_cells.extract_surface()
    
    # Get oriented bounding box
    obb_mesh = create_oriented_bbox(cluster_surface)
    
    # Add to plotter
    plotter.add_mesh(
        obb_mesh,
        color='red',
        line_width=3,
        label=f'Cluster {cluster_id} OBB'
    )