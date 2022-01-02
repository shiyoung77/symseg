import os
import argparse

import numpy as np
import open3d as o3d

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--dataset", type=str, default='dataset')
    parser.add_argument("-v", "--video", type=str, default="0001")
    args = parser.parse_args()

    video_folder = os.path.join(args.dataset, args.video)

    pcd = o3d.io.read_point_cloud(os.path.join(video_folder, 'sampled_pcd/cloud.pcd'))
    o3d.visualization.draw_geometries([pcd])

    segments_file = os.path.join(video_folder, 'sampled_pcd/results/segments.txt')
    with open(segments_file, 'r') as f:
        segments = [line.strip().split() for line in f.readlines()]

    for i, segment in enumerate(segments):
        segments[i] = [int(idx) for idx in segment]

    random_colors = np.random.random((100, 3))
    segment_pcds = [pcd.select_by_index(segment).paint_uniform_color(random_colors[i]) for i, segment in enumerate(segments)]
    print(f"number of segments: {len(segment_pcds)}")
    o3d.visualization.draw_geometries(segment_pcds)
