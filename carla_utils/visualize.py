from argparse import ArgumentParser
from .recording import parse
from .visualization.renderer import Renderer, bounding_view_matrix, const_view_matrix, follow_view_matrix

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('recording')
    parser.add_argument('output')
    parser.add_argument('--width', type=int, default=512)
    parser.add_argument('--height', type=int, default=512)
    parser.add_argument('--fps', type=int, default=20)
    parser.add_argument('--max-frames', type=int)
    parser.add_argument('-v', '--view_matrix', type=str, default='bounding_view_matrix()')
    args = parser.parse_args()
    world_map, frames = parse(args.recording)

    if args.max_frames is not None:
        frames = frames[:args.max_frames]

    view_matrix = eval(args.view_matrix, dict(bounding_view_matrix=bounding_view_matrix, const_view_matrix=const_view_matrix, follow_view_matrix=follow_view_matrix))

    renderer = Renderer(args.width, args.height)
    renderer.save_video(world_map, frames, args.output, view_matrix=view_matrix, fps=args.fps)
