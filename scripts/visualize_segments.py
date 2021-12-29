import numpy as np
import open3d as o3d

pcd = o3d.io.read_point_cloud('../sample_scene/cloud.ply')
o3d.visualization.draw_geometries([pcd])

segments_file = '../sample_scene/results/segments.txt'
with open(segments_file, 'r') as f:
    segments = [line.strip().split() for line in f.readlines()]

for i, segment in enumerate(segments):
    segments[i] = [int(idx) for idx in segment]

random_colors = np.random.random((100, 3))
segment_pcds = [pcd.select_by_index(segment).paint_uniform_color(random_colors[i]) for i, segment in enumerate(segments)]
print(f"number of segments: {len(segment_pcds)}")
o3d.visualization.draw_geometries(segment_pcds)