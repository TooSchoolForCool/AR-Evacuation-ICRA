import argparse

def arg_parser():
    """Argument Parser
    Parse arguments from command line, and perform error checking
    Returns:
        An argument object which contains arguments from cmd line
    """
    parser = argparse.ArgumentParser(prog='Generate grid map')

    parser.add_argument(
        "--src",
        dest="src",
        type=str,
        required=True,
        help="Original map source"
    )
    parser.add_argument(
        "--scaler",
        dest="scaler",
        type=int,
        default=10,
        help="Define the size of each grid nxn pixels"
    )
    
    args = parser.parse_args()

    return args