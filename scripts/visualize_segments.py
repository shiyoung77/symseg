import os
import argparse

import numpy as np
import open3d as o3d
import networkx as nx


def filter_segments(segments):
    segments = [set(segment) for segment in segments]
    N = len(segments)
    print(f"number of segments before filtering: {N}")

    G = nx.Graph()
    G.add_nodes_from(range(N))

    for i in range(len(segments)):
        for j in range(i, len(segments)):
            n1 = len(segments[i])
            n2 = len(segments[j])
            iou = len(segments[i] & segments[j]) / (n1 + n2)
            if iou > 0.3:
                G.add_edge(i, j)

    filtered_segments = []

    # connected components
    cc_list = list(nx.connected_components(G))
    for cc in cc_list:
        cc = list(cc)
        indices = segments[cc[0]]
        for i in range(1, len(cc)):
            indices &= segments[cc[i]]
        filtered_segments.append(list(indices))

    print(f"number of segments after filtering: {len(filtered_segments)}")
    return filtered_segments


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # parser.add_argument("-d", "--dataset", type=str, default='dataset')
    # parser.add_argument("-d", "--dataset", type=str, default='/home/lsy/dataset/ycb_video')
    parser.add_argument("-d", "--dataset", type=str, default='/home/lsy/dataset/icra22_custom_ycb')
    parser.add_argument("-v", "--video", type=str, default="0020")
    args = parser.parse_args()

    video_folder = os.path.join(args.dataset, args.video)

    pcd = o3d.io.read_point_cloud(os.path.join(video_folder, 'sampled_pcd/cloud.pcd'))
    # o3d.visualization.draw_geometries([pcd])

    segments_file = os.path.join(video_folder, 'sampled_pcd/results/segments.txt')
    with open(segments_file, 'r') as f:
        segments = [line.strip().split() for line in f.readlines()]

    for i, segment in enumerate(segments):
        segments[i] = [int(idx) for idx in segment]

    segments = filter_segments(segments)

    segmented_indices = set()
    for segment in segments:
        segmented_indices = segmented_indices.union(segment)
    all_indices = set(range(len(pcd.points)))
    unsegmented_indices = all_indices - segmented_indices
    print(f"{len(segmented_indices) = }")
    print(f"{len(unsegmented_indices) = }")

    random_colors = np.random.random((100, 3))
    segment_pcds = [pcd.select_by_index(segment).paint_uniform_color(random_colors[i]) for i, segment in enumerate(segments)]
    unsegmented_pcd = pcd.select_by_index(list(unsegmented_indices)).paint_uniform_color([0, 0, 0])
    print(f"number of segments: {len(segment_pcds)}")

    o3d.visualization.draw_geometries(segment_pcds + [unsegmented_pcd])
