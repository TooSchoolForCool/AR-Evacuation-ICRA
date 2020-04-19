from utils import arg_parser
from grid_map_generator import GridMapGenerator


if __name__ == "__main__":
    args = arg_parser()

    generator = GridMapGenerator()
    generator.generate(args.src, args.scaler)

